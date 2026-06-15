"""GNSS-denied visual localization baseline.

This script simulates visual navigation without GNSS.

Given:
    1. a preprocessed reference database built from GNSS-labeled videos,
    2. a query video, treated as GNSS-denied,

it:
    1. extracts query frames,
    2. computes ORB features,
    3. matches each query frame against the reference database frames,
    4. uses the best matched reference frame's known camera-center coordinate
       as the estimated location of the query frame.

Optionally, if a query SRT is provided, it is used only for evaluation.
The SRT is not used for visual matching.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import pandas as pd

from src.geo_utils import haversine_m
from src.video_utils import extract_frames

# Optional evaluation imports.
from src.srt_parser import parse_srt
from preliminary_experiment import (
    prepare_telemetry_for_experiment,
    add_clean_center_projection,
)


def read_image_gray(path: str, max_width: int) -> Optional[np.ndarray]:
    """Read image as grayscale and optionally resize it for faster matching."""
    img = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)

    if img is None:
        return None

    if max_width > 0 and img.shape[1] > max_width:
        scale = max_width / img.shape[1]
        new_h = int(img.shape[0] * scale)
        img = cv2.resize(img, (max_width, new_h), interpolation=cv2.INTER_AREA)

    return img


def compute_orb_features(
    image: np.ndarray,
    nfeatures: int,
) -> tuple[list[cv2.KeyPoint], Optional[np.ndarray]]:
    """Compute ORB keypoints and descriptors."""
    orb = cv2.ORB_create(nfeatures=nfeatures)
    keypoints, descriptors = orb.detectAndCompute(image, None)
    return keypoints, descriptors


def ratio_match_orb(
    query_desc: Optional[np.ndarray],
    ref_desc: Optional[np.ndarray],
    ratio: float,
) -> list[cv2.DMatch]:
    """Match ORB descriptors using Lowe ratio test."""
    if query_desc is None or ref_desc is None:
        return []

    if len(query_desc) < 2 or len(ref_desc) < 2:
        return []

    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

    try:
        raw_matches = bf.knnMatch(query_desc, ref_desc, k=2)
    except cv2.error:
        return []

    good_matches = []

    for pair in raw_matches:
        if len(pair) < 2:
            continue

        m, n = pair

        if m.distance < ratio * n.distance:
            good_matches.append(m)

    return good_matches


def homography_inlier_count(
    query_kp: list[cv2.KeyPoint],
    ref_kp: list[cv2.KeyPoint],
    matches: list[cv2.DMatch],
) -> int:
    """Estimate homography and return number of RANSAC inliers."""
    if len(matches) < 8:
        return 0

    query_pts = np.float32([query_kp[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
    ref_pts = np.float32([ref_kp[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)

    try:
        _, mask = cv2.findHomography(query_pts, ref_pts, cv2.RANSAC, 5.0)
    except cv2.error:
        return 0

    if mask is None:
        return 0

    return int(mask.sum())


def load_reference_features(
    reference_db: pd.DataFrame,
    args: argparse.Namespace,
) -> list[dict]:
    """Load reference images and pre-compute ORB descriptors."""
    features = []

    if args.max_reference_frames is not None:
        reference_db = reference_db.head(args.max_reference_frames).copy()

    print(f"Loading reference features from {len(reference_db)} frames...")

    for idx, row in reference_db.iterrows():
        image_path = str(row["image_path"])

        img = read_image_gray(image_path, args.max_image_width)

        if img is None:
            continue

        kp, desc = compute_orb_features(img, args.orb_features)

        if desc is None or len(kp) == 0:
            continue

        features.append(
            {
                "reference_index": idx,
                "image_path": image_path,
                "source_name": row.get("source_name", ""),
                "center_latitude": row["center_latitude"],
                "center_longitude": row["center_longitude"],
                "drone_latitude": row.get("latitude", np.nan),
                "drone_longitude": row.get("longitude", np.nan),
                "timestamp_s": row.get("timestamp_s", np.nan),
                "keypoints": kp,
                "descriptors": desc,
            }
        )

    print(f"Loaded usable reference frames: {len(features)}")

    if not features:
        raise RuntimeError("No usable reference features were loaded.")

    return features


def find_best_reference_match(
    query_image_path: str,
    reference_features: list[dict],
    args: argparse.Namespace,
) -> tuple[Optional[dict], dict, list[cv2.DMatch], list[cv2.KeyPoint]]:
    """Find the best reference frame for a single query frame."""
    query_img = read_image_gray(query_image_path, args.max_image_width)

    if query_img is None:
        return None, {}, [], []

    query_kp, query_desc = compute_orb_features(query_img, args.orb_features)

    if query_desc is None or len(query_kp) == 0:
        return None, {}, [], query_kp

    best_ref = None
    best_stats = {
        "match_score": 0.0,
        "good_matches": 0,
        "mean_distance": np.nan,
        "homography_inliers": 0,
    }
    best_matches = []

    for ref in reference_features:
        matches = ratio_match_orb(query_desc, ref["descriptors"], args.ratio)

        good_count = len(matches)

        if good_count == 0:
            continue

        distances = [m.distance for m in matches]
        mean_distance = float(np.mean(distances))

        inliers = 0
        if args.use_homography_score:
            inliers = homography_inlier_count(query_kp, ref["keypoints"], matches)
            score = inliers * 10 + good_count
        else:
            score = good_count - 0.01 * mean_distance

        if score > best_stats["match_score"]:
            best_ref = ref
            best_matches = matches
            best_stats = {
                "match_score": float(score),
                "good_matches": int(good_count),
                "mean_distance": mean_distance,
                "homography_inliers": int(inliers),
            }

    return best_ref, best_stats, best_matches, query_kp


def save_match_visualization(
    query_image_path: str,
    best_ref: dict,
    query_kp: list[cv2.KeyPoint],
    matches: list[cv2.DMatch],
    out_path: Path,
    args: argparse.Namespace,
) -> None:
    """Save an image showing query/reference feature matches."""
    if best_ref is None or not matches:
        return

    query_img = read_image_gray(query_image_path, args.max_image_width)
    ref_img = read_image_gray(best_ref["image_path"], args.max_image_width)

    if query_img is None or ref_img is None:
        return

    drawn = cv2.drawMatches(
        query_img,
        query_kp,
        ref_img,
        best_ref["keypoints"],
        matches[: args.max_draw_matches],
        None,
        flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS,
    )

    cv2.imwrite(str(out_path), drawn)


def add_optional_ground_truth(
    results: pd.DataFrame,
    args: argparse.Namespace,
) -> pd.DataFrame:
    """If query SRT is provided, add ground-truth projected coordinates.

    This is used only for evaluation. It is not used by the matching algorithm.
    """
    if args.query_srt is None:
        return results

    telemetry = parse_srt(args.query_srt)

    eval_args = argparse.Namespace(
        fallback_altitude_m=args.fallback_altitude_m,
        min_altitude=args.min_altitude,
        frame_step=args.frame_step,
        telemetry_step=args.frame_step,
        heading_from_gnss=True,
        min_heading_motion_m=args.min_heading_motion_m,
        smooth_heading_window=args.smooth_heading_window,
        camera_angle=args.camera_angle,
        fallback_pitch_deg=args.fallback_pitch_deg,
        pitch_convention=args.pitch_convention,
    )

    prepared = prepare_telemetry_for_experiment(telemetry, eval_args)
    projected = add_clean_center_projection(prepared, eval_args)

    gt = projected[
        [
            "timestamp_s",
            "latitude",
            "longitude",
            "center_latitude",
            "center_longitude",
            "used_altitude_m",
            "used_heading_deg",
        ]
    ].copy()

    gt = gt.rename(
        columns={
            "latitude": "gt_drone_latitude",
            "longitude": "gt_drone_longitude",
            "center_latitude": "gt_center_latitude",
            "center_longitude": "gt_center_longitude",
            "used_altitude_m": "gt_altitude_m",
            "used_heading_deg": "gt_heading_deg",
        }
    )

    merged = pd.merge_asof(
        results.sort_values("query_timestamp_s"),
        gt.sort_values("timestamp_s"),
        left_on="query_timestamp_s",
        right_on="timestamp_s",
        direction="nearest",
        tolerance=args.sync_tolerance_s,
    )

    errors = []

    for _, row in merged.iterrows():
        if pd.isna(row.get("gt_center_latitude")):
            errors.append(np.nan)
            continue

        err = haversine_m(
            row["estimated_center_latitude"],
            row["estimated_center_longitude"],
            row["gt_center_latitude"],
            row["gt_center_longitude"],
        )
        errors.append(err)

    merged["center_error_m"] = errors

    return merged


def write_summary(results: pd.DataFrame, outdir: Path, args: argparse.Namespace) -> None:
    """Write a text summary of matching results."""
    lines = []
    lines.append("GNSS-denied visual matching summary")
    lines.append("=" * 50)
    lines.append("")
    lines.append(f"Reference database: {args.reference_db}")
    lines.append(f"Query video: {args.query_video}")
    lines.append(f"Query SRT for evaluation: {args.query_srt}")
    lines.append(f"Query frames processed: {len(results)}")
    lines.append(f"Frame step: {args.frame_step}")
    lines.append(f"ORB features: {args.orb_features}")
    lines.append(f"Ratio test: {args.ratio}")
    lines.append(f"Use homography score: {args.use_homography_score}")
    lines.append("")

    if "best_source_name" in results.columns:
        lines.append("Best matched source video counts:")
        lines.append(str(results["best_source_name"].value_counts()))
        lines.append("")

    lines.append("Match statistics:")
    lines.append(str(results[["match_score", "good_matches", "mean_distance", "homography_inliers"]].describe()))
    lines.append("")

    if "center_error_m" in results.columns:
        lines.append("Evaluation error statistics, meters:")
        lines.append(str(results["center_error_m"].describe()))
        lines.append("")

    (outdir / "match_summary.txt").write_text("\n".join(lines), encoding="utf-8")


def main(args: argparse.Namespace) -> None:
    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    reference_db = pd.read_csv(args.reference_db)

    required_cols = {"image_path", "center_latitude", "center_longitude"}
    missing = required_cols - set(reference_db.columns)

    if missing:
        raise RuntimeError(f"Reference DB is missing required columns: {missing}")

    if args.exclude_source:
        reference_db = reference_db[reference_db["source_name"] != args.exclude_source].copy()

    reference_features = load_reference_features(reference_db, args)

    query_frames_dir = outdir / "query_frames"
    query_frames = extract_frames(
        args.query_video,
        query_frames_dir,
        frame_step=args.frame_step,
        max_frames=args.max_query_frames,
    )

    if query_frames.empty:
        raise RuntimeError("No query frames were extracted.")

    match_examples_dir = outdir / "matched_examples"
    match_examples_dir.mkdir(parents=True, exist_ok=True)

    results = []

    print(f"\nMatching {len(query_frames)} query frames...")

    for i, row in query_frames.iterrows():
        query_image_path = str(row["image_path"])

        best_ref, stats, matches, query_kp = find_best_reference_match(
            query_image_path,
            reference_features,
            args,
        )

        if best_ref is None:
            result = {
                "query_frame_index": row["frame_index"],
                "query_timestamp_s": row["timestamp_s"],
                "query_image_path": query_image_path,
                "best_reference_image_path": None,
                "best_source_name": None,
                "estimated_center_latitude": np.nan,
                "estimated_center_longitude": np.nan,
                "estimated_drone_latitude": np.nan,
                "estimated_drone_longitude": np.nan,
                "match_score": 0.0,
                "good_matches": 0,
                "mean_distance": np.nan,
                "homography_inliers": 0,
            }
        else:
            result = {
                "query_frame_index": row["frame_index"],
                "query_timestamp_s": row["timestamp_s"],
                "query_image_path": query_image_path,
                "best_reference_image_path": best_ref["image_path"],
                "best_source_name": best_ref["source_name"],
                "estimated_center_latitude": best_ref["center_latitude"],
                "estimated_center_longitude": best_ref["center_longitude"],
                "estimated_drone_latitude": best_ref["drone_latitude"],
                "estimated_drone_longitude": best_ref["drone_longitude"],
                **stats,
            }

            if i < args.save_match_images:
                match_path = match_examples_dir / f"match_{i:04d}_{best_ref['source_name']}.jpg"
                save_match_visualization(
                    query_image_path,
                    best_ref,
                    query_kp,
                    matches,
                    match_path,
                    args,
                )

        results.append(result)

        if (i + 1) % 10 == 0:
            print(f"Matched {i + 1}/{len(query_frames)} query frames")

    results_df = pd.DataFrame(results)
    results_df = add_optional_ground_truth(results_df, args)

    results_csv = outdir / "visual_localization_results.csv"
    results_df.to_csv(results_csv, index=False)

    write_summary(results_df, outdir, args)

    print("\nDone.")
    print(f"Results CSV: {results_csv}")
    print(f"Summary: {outdir / 'match_summary.txt'}")
    print(f"Match examples: {match_examples_dir}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run GNSS-denied visual matching baseline.")

    parser.add_argument("--reference-db", required=True, help="Reference database CSV")
    parser.add_argument("--query-video", required=True, help="GNSS-denied query video")
    parser.add_argument("--query-srt", default=None, help="Optional query SRT, used only for evaluation")
    parser.add_argument("--outdir", default="outputs/visual_matching")

    parser.add_argument("--frame-step", type=int, default=60)
    parser.add_argument("--max-query-frames", type=int, default=None)
    parser.add_argument("--max-reference-frames", type=int, default=None)

    parser.add_argument("--orb-features", type=int, default=2000)
    parser.add_argument("--ratio", type=float, default=0.75)
    parser.add_argument("--max-image-width", type=int, default=640)

    parser.add_argument(
        "--use-homography-score",
        action="store_true",
        help="Use RANSAC homography inliers as main matching score.",
    )

    parser.add_argument("--save-match-images", type=int, default=20)
    parser.add_argument("--max-draw-matches", type=int, default=50)
    parser.add_argument("--exclude-source", default=None)

    # Evaluation/projection options for optional query SRT.
    parser.add_argument("--camera-angle", type=float, default=60.0)
    parser.add_argument("--min-altitude", type=float, default=20.0)
    parser.add_argument("--min-heading-motion-m", type=float, default=1.0)
    parser.add_argument("--smooth-heading-window", type=int, default=5)
    parser.add_argument("--sync-tolerance-s", type=float, default=2.0)

    parser.add_argument("--fallback-pitch-deg", type=float, default=-60.0)
    parser.add_argument("--fallback-altitude-m", type=float, default=119.0)
    parser.add_argument("--pitch-convention", default="dji", choices=["dji", "positive_down"])

    main(parser.parse_args())