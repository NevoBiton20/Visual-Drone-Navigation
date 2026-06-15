"""Build the preprocessing/reference database for GNSS-denied visual navigation.

This script processes GNSS-labeled drone videos and SRT telemetry files.

For each video/SRT pair, it:
    1. extracts sampled video frames,
    2. parses SRT telemetry,
    3. filters unstable low-altitude frames,
    4. estimates heading from GNSS when heading/yaw is missing,
    5. projects the camera-center point onto the ground,
    6. synchronizes frames with telemetry,
    7. writes a reference database CSV.

This database is later used by localize_by_visual_matching.py.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd

from src.geo_utils import bearing_deg, destination_point, haversine_m
from src.projection_utils import (
    choose_altitude_m,
    depression_angle_from_pitch,
    fill_missing_heading,
    center_ground_offset_m,
)
from src.srt_parser import parse_srt
from src.video_utils import extract_frames


def circular_rolling_mean_deg(headings: pd.Series, window: int) -> pd.Series:
    """Smooth heading angles using circular mean."""
    if window <= 1 or len(headings) == 0:
        return headings % 360

    radians = np.deg2rad(headings.astype(float))

    sin_mean = pd.Series(np.sin(radians)).rolling(
        window=window,
        center=True,
        min_periods=1,
    ).mean()

    cos_mean = pd.Series(np.cos(radians)).rolling(
        window=window,
        center=True,
        min_periods=1,
    ).mean()

    smoothed = np.rad2deg(np.arctan2(sin_mean, cos_mean)) % 360
    return pd.Series(smoothed, index=headings.index)


def estimate_heading_from_gnss(
    df: pd.DataFrame,
    min_motion_m: float = 1.0,
) -> pd.Series:
    """Estimate heading from GNSS movement.

    Many SRT files do not include yaw/heading. Consecutive frames can also have
    almost identical GPS coordinates, so this function searches for a future or
    previous point that is at least min_motion_m meters away.
    """
    if len(df) == 0:
        return pd.Series(dtype=float)

    if len(df) == 1:
        return pd.Series([0.0], index=df.index)

    headings = []

    for i in range(len(df)):
        lat1 = float(df.iloc[i]["latitude"])
        lon1 = float(df.iloc[i]["longitude"])

        chosen_heading = np.nan

        # Prefer a future point.
        for j in range(i + 1, len(df)):
            lat2 = float(df.iloc[j]["latitude"])
            lon2 = float(df.iloc[j]["longitude"])

            if haversine_m(lat1, lon1, lat2, lon2) >= min_motion_m:
                chosen_heading = bearing_deg(lat1, lon1, lat2, lon2)
                break

        # If no future point exists, use a previous point.
        if pd.isna(chosen_heading):
            for j in range(i - 1, -1, -1):
                lat0 = float(df.iloc[j]["latitude"])
                lon0 = float(df.iloc[j]["longitude"])

                if haversine_m(lat0, lon0, lat1, lon1) >= min_motion_m:
                    chosen_heading = bearing_deg(lat0, lon0, lat1, lon1)
                    break

        headings.append(chosen_heading)

    heading_series = pd.Series(headings, index=df.index)
    heading_series = heading_series.ffill().bfill().fillna(0.0)

    return heading_series % 360


def prepare_telemetry(
    telemetry: pd.DataFrame,
    args: argparse.Namespace,
) -> pd.DataFrame:
    """Clean, filter, sample, and project telemetry."""
    df = telemetry.copy()

    df = df[pd.notna(df["latitude"]) & pd.notna(df["longitude"])].copy()

    if df.empty:
        raise RuntimeError("No valid latitude/longitude values were found in the SRT file.")

    df["used_altitude_m"] = df.apply(
        lambda row: choose_altitude_m(row, args.fallback_altitude_m),
        axis=1,
    )

    if args.min_altitude > 0:
        df = df[df["used_altitude_m"] >= args.min_altitude].copy()

    if df.empty:
        raise RuntimeError(
            "All telemetry rows were removed by --min-altitude. "
            "Try using a lower value."
        )

    telemetry_step = args.telemetry_step
    if telemetry_step is None:
        telemetry_step = max(1, args.frame_step)

    df = df.iloc[::telemetry_step].copy().reset_index(drop=True)

    if args.heading_from_gnss:
        heading = estimate_heading_from_gnss(
            df,
            min_motion_m=args.min_heading_motion_m,
        )
    else:
        heading = fill_missing_heading(df)

    heading = circular_rolling_mean_deg(heading, args.smooth_heading_window)
    df["used_heading_deg"] = heading.values

    center_lats = []
    center_lons = []
    depressions = []
    offsets = []

    for _, row in df.iterrows():
        lat = float(row["latitude"])
        lon = float(row["longitude"])
        altitude_m = float(row["used_altitude_m"])

        if args.camera_angle is not None:
            depression = float(args.camera_angle)
        else:
            pitch = row.get("gimbal_pitch_deg", np.nan)
            depression = depression_angle_from_pitch(
                pitch,
                fallback_pitch_deg=args.fallback_pitch_deg,
                convention=args.pitch_convention,
            )

        depression = max(1e-3, min(89.999, depression))
        offset_m = center_ground_offset_m(altitude_m, depression)

        center_lat, center_lon = destination_point(
            lat,
            lon,
            float(row["used_heading_deg"]),
            offset_m,
        )

        center_lats.append(center_lat)
        center_lons.append(center_lon)
        depressions.append(depression)
        offsets.append(offset_m)

    df["camera_depression_deg"] = depressions
    df["center_offset_m"] = offsets
    df["center_latitude"] = center_lats
    df["center_longitude"] = center_lons

    return df


def process_pair(row: pd.Series, out_root: Path, args: argparse.Namespace) -> pd.DataFrame:
    """Process one video/SRT pair and return its synchronized reference rows."""
    name = row.get("name")

    if pd.isna(name) or not str(name).strip():
        name = Path(row["video_path"]).stem

    name = str(name)
    video_path = str(row["video_path"])
    srt_path = str(row["srt_path"])

    pair_dir = out_root / name
    frames_dir = pair_dir / "frames"
    pair_dir.mkdir(parents=True, exist_ok=True)

    print(f"\nProcessing pair: {name}")
    print(f"Video: {video_path}")
    print(f"SRT:   {srt_path}")

    frames = extract_frames(
        video_path,
        frames_dir,
        frame_step=args.frame_step,
        max_frames=args.max_frames,
    )

    telemetry = parse_srt(srt_path)
    projected = prepare_telemetry(telemetry, args)

    if frames.empty:
        raise RuntimeError(f"No frames were extracted from {video_path}")

    if projected.empty:
        raise RuntimeError(f"No projected telemetry was created from {srt_path}")

    synced = pd.merge_asof(
        frames.sort_values("timestamp_s"),
        projected.sort_values("timestamp_s"),
        on="timestamp_s",
        direction="nearest",
        tolerance=args.sync_tolerance_s,
    )

    # Keep only frames that successfully received projected coordinates.
    synced = synced[pd.notna(synced["center_latitude"]) & pd.notna(synced["center_longitude"])].copy()

    synced["source_name"] = name
    synced["source_video_path"] = video_path
    synced["source_srt_path"] = srt_path

    per_video_csv = pair_dir / "reference_frames.csv"
    synced.to_csv(per_video_csv, index=False)

    telemetry_csv = pair_dir / "projected_telemetry.csv"
    projected.to_csv(telemetry_csv, index=False)

    summary_path = pair_dir / "reference_summary.txt"
    summary_path.write_text(
        "\n".join([
            f"Reference database summary for {name}",
            "=" * 50,
            f"Video: {video_path}",
            f"SRT: {srt_path}",
            f"Extracted frames: {len(frames)}",
            f"Projected telemetry samples: {len(projected)}",
            f"Synchronized reference rows: {len(synced)}",
            f"Frame step: {args.frame_step}",
            f"Telemetry step: {args.telemetry_step if args.telemetry_step is not None else args.frame_step}",
            f"Minimum altitude: {args.min_altitude}",
            f"Camera angle: {args.camera_angle}",
            f"Heading from GNSS: {args.heading_from_gnss}",
            f"Heading smoothing window: {args.smooth_heading_window}",
            "",
            "Projection statistics:",
            str(synced[[
                "used_altitude_m",
                "used_heading_deg",
                "camera_depression_deg",
                "center_offset_m",
            ]].describe()),
        ]),
        encoding="utf-8",
    )

    print(f"Per-video reference CSV: {per_video_csv}")
    print(f"Synchronized reference rows: {len(synced)}")

    return synced


def main(args: argparse.Namespace) -> None:
    out_root = Path(args.outdir)
    out_root.mkdir(parents=True, exist_ok=True)

    pairs = pd.read_csv(args.pairs)

    required_cols = {"video_path", "srt_path"}
    missing = required_cols - set(pairs.columns)

    if missing:
        raise RuntimeError(
            f"Pairs CSV is missing required columns: {missing}. "
            "Expected columns: name,video_path,srt_path"
        )

    all_rows = []

    for _, row in pairs.iterrows():
        all_rows.append(process_pair(row, out_root, args))

    if not all_rows:
        raise RuntimeError("No video/SRT pairs were processed.")

    reference_db = pd.concat(all_rows, ignore_index=True)

    reference_db_path = out_root / "reference_database.csv"
    reference_db.to_csv(reference_db_path, index=False)

    print("\nDone.")
    print(f"Reference database written to: {reference_db_path}")
    print(f"Total rows: {len(reference_db)}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Build visual navigation reference database.")

    parser.add_argument("--pairs", required=True, help="CSV with columns: name,video_path,srt_path")
    parser.add_argument("--outdir", default="data/processed/reference_database")

    parser.add_argument("--frame-step", type=int, default=30)
    parser.add_argument(
        "--telemetry-step",
        type=int,
        default=None,
        help="Use one telemetry row every N rows. If omitted, uses --frame-step.",
    )
    parser.add_argument("--max-frames", type=int, default=None)

    parser.add_argument(
        "--min-altitude",
        type=float,
        default=0.0,
        help="Ignore telemetry rows below this altitude in meters.",
    )

    parser.add_argument(
        "--camera-angle",
        type=float,
        default=None,
        help="Manual camera depression angle below horizon. Example: --camera-angle 60.",
    )

    parser.add_argument(
        "--heading-from-gnss",
        action="store_true",
        help="Estimate heading from GNSS trajectory instead of SRT yaw/heading.",
    )

    parser.add_argument(
        "--min-heading-motion-m",
        type=float,
        default=1.0,
        help="Minimum GNSS movement in meters used for heading estimation.",
    )

    parser.add_argument(
        "--smooth-heading-window",
        type=int,
        default=5,
        help="Circular rolling smoothing window for heading.",
    )

    parser.add_argument(
        "--sync-tolerance-s",
        type=float,
        default=1.0,
        help="Maximum time difference allowed between frame timestamp and telemetry timestamp.",
    )

    parser.add_argument("--fallback-pitch-deg", type=float, default=-60.0)
    parser.add_argument("--fallback-altitude-m", type=float, default=119.0)
    parser.add_argument("--pitch-convention", default="dji", choices=["dji", "positive_down"])

    main(parser.parse_args())