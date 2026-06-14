"""GNSS-denied visual localization baseline.

This script takes query frames from a test video and searches a previously built
reference database. It uses ORB + homography in the baseline version.

The result is not meant to be final research-grade localization; it is a clear,
explainable baseline that demonstrates the assignment's navigation stage.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import pandas as pd
from tqdm import tqdm

from src.feature_matching import pair_match_score
from src.video_utils import extract_frames


def localize_query_frame(query_img: str, reference_db: pd.DataFrame, top_k_scan: int, nfeatures: int) -> dict:
    # Practical shortcut: for small assignments, scan all sampled reference frames.
    # For larger data, replace this with image embeddings + nearest-neighbor search.
    best = None
    candidates = reference_db.dropna(subset=["image_path", "center_latitude", "center_longitude"])
    if top_k_scan > 0:
        candidates = candidates.head(top_k_scan)

    for _, ref in candidates.iterrows():
        try:
            score_info = pair_match_score(query_img, ref["image_path"], nfeatures=nfeatures)
        except Exception:
            continue
        row = {
            "query_image_path": query_img,
            "matched_reference_image_path": ref["image_path"],
            "source_name": ref.get("source_name"),
            "estimated_center_latitude": ref.get("center_latitude"),
            "estimated_center_longitude": ref.get("center_longitude"),
            "estimated_drone_latitude": ref.get("latitude"),
            "estimated_drone_longitude": ref.get("longitude"),
            "num_matches": score_info.get("num_matches"),
            "num_inliers": score_info.get("num_inliers"),
            "dx_px": score_info.get("dx_px"),
            "dy_px": score_info.get("dy_px"),
            "score": score_info.get("score", 0),
        }
        if best is None or row["score"] > best["score"]:
            best = row

    return best or {"query_image_path": query_img, "score": 0}


def main(args: argparse.Namespace) -> None:
    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    reference_db = pd.read_csv(args.reference_db)
    frames = extract_frames(args.query_video, outdir / "query_frames", frame_step=args.frame_step, max_frames=args.max_frames)

    results = []
    for _, row in tqdm(frames.iterrows(), total=len(frames), desc="Localizing query frames"):
        result = localize_query_frame(row["image_path"], reference_db, args.top_k_scan, args.orb_features)
        result.update({
            "query_frame_index": row["frame_index"],
            "query_timestamp_s": row["timestamp_s"],
        })
        results.append(result)

    results_df = pd.DataFrame(results)
    out_csv = outdir / "visual_localization_results.csv"
    results_df.to_csv(out_csv, index=False)
    print(f"Wrote {len(results_df)} localization rows to {out_csv}")
    print("Tip: if matching is weak, increase overlap between training/test videos or use LightGlue/SuperPoint.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Localize GNSS-denied query video by matching against reference frames")
    parser.add_argument("--reference-db", required=True, help="reference_database.csv from build_reference_database.py")
    parser.add_argument("--query-video", required=True, help="GNSS-denied video, or a video treated as GNSS-denied")
    parser.add_argument("--outdir", default="outputs/visual_matching")
    parser.add_argument("--frame-step", type=int, default=30)
    parser.add_argument("--max-frames", type=int, default=30)
    parser.add_argument("--top-k-scan", type=int, default=0, help="0 means scan all reference rows; use small number for speed tests")
    parser.add_argument("--orb-features", type=int, default=3000)
    main(parser.parse_args())
