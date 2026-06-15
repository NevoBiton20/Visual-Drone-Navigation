"""Preliminary experiment for Ex1: geometry-only camera-center projection.

This script parses drone SRT telemetry and estimates the geographic coordinate
of the ground point seen at the center of the video frame.

Main idea:
    drone GNSS + altitude + camera angle + heading
        -> project camera-center ray to flat ground
        -> compare drone GNSS path with projected camera-center path

Clean experiment support:
    - filter low-altitude frames
    - sample telemetry frames
    - use known camera depression angle, e.g. 60 degrees
    - estimate heading from GNSS trajectory when yaw/heading is missing
    - smooth heading to reduce noisy projections during turns
"""

from __future__ import annotations

import argparse
import math
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
from src.plotting import plot_paths, save_kml


def circular_rolling_mean_deg(headings: pd.Series, window: int) -> pd.Series:
    """Smooth heading angles correctly using circular mean.

    A normal mean fails for angles around 0/360.
    Example: mean(359, 1) should be 0, not 180.
    """
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

    The raw SRT may contain many consecutive frames with almost identical GPS
    coordinates. In that case, using immediate consecutive points can produce
    unstable or meaningless headings. This function searches forward/backward
    until the drone has moved at least min_motion_m meters.
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

        # Prefer a future point far enough away.
        for j in range(i + 1, len(df)):
            lat2 = float(df.iloc[j]["latitude"])
            lon2 = float(df.iloc[j]["longitude"])
            dist = haversine_m(lat1, lon1, lat2, lon2)

            if dist >= min_motion_m:
                chosen_heading = bearing_deg(lat1, lon1, lat2, lon2)
                break

        # If no future point exists, use a previous point.
        if pd.isna(chosen_heading):
            for j in range(i - 1, -1, -1):
                lat0 = float(df.iloc[j]["latitude"])
                lon0 = float(df.iloc[j]["longitude"])
                dist = haversine_m(lat0, lon0, lat1, lon1)

                if dist >= min_motion_m:
                    chosen_heading = bearing_deg(lat0, lon0, lat1, lon1)
                    break

        headings.append(chosen_heading)

    heading_series = pd.Series(headings, index=df.index)
    heading_series = heading_series.ffill().bfill().fillna(0.0)
    return heading_series % 360


def prepare_telemetry_for_experiment(
    telemetry: pd.DataFrame,
    args: argparse.Namespace,
) -> pd.DataFrame:
    """Filter and sample telemetry before projection."""
    df = telemetry.copy()

    # Keep only rows with valid coordinates.
    df = df[pd.notna(df["latitude"]) & pd.notna(df["longitude"])].copy()

    if df.empty:
        raise RuntimeError("No valid latitude/longitude rows found in SRT.")

    # Compute altitude before filtering.
    df["used_altitude_m"] = df.apply(
        lambda row: choose_altitude_m(row, args.fallback_altitude_m),
        axis=1,
    )

    # Remove takeoff/landing/unstable low-altitude frames if requested.
    if args.min_altitude > 0:
        df = df[df["used_altitude_m"] >= args.min_altitude].copy()

    if df.empty:
        raise RuntimeError(
            "All telemetry rows were removed by --min-altitude. "
            "Try a lower value."
        )

    # Sample telemetry to make plots cleaner and heading estimation more stable.
    telemetry_step = args.telemetry_step
    if telemetry_step is None:
        telemetry_step = max(1, args.frame_step)

    df = df.iloc[::telemetry_step].copy().reset_index(drop=True)

    return df


def add_clean_center_projection(
    df: pd.DataFrame,
    args: argparse.Namespace,
) -> pd.DataFrame:
    """Add clean heading and camera-center ground projection columns."""
    out = df.copy()

    # Heading source.
    if args.heading_from_gnss:
        heading = estimate_heading_from_gnss(
            out,
            min_motion_m=args.min_heading_motion_m,
        )
    else:
        heading = fill_missing_heading(out)

    heading = circular_rolling_mean_deg(heading, args.smooth_heading_window)
    out["used_heading_deg"] = heading.values

    center_lats = []
    center_lons = []
    depressions = []
    offsets = []

    for _, row in out.iterrows():
        lat = float(row["latitude"])
        lon = float(row["longitude"])
        altitude_m = float(row["used_altitude_m"])

        # Camera depression angle.
        # If --camera-angle is supplied, it is used as a positive depression
        # angle below the horizon. Example: --camera-angle 60.
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

    out["camera_depression_deg"] = depressions
    out["center_offset_m"] = offsets
    out["center_latitude"] = center_lats
    out["center_longitude"] = center_lons

    return out


def write_experiment_summary(projected: pd.DataFrame, outdir: Path, args: argparse.Namespace) -> None:
    """Write a small text summary of the experiment parameters and statistics."""
    summary_path = outdir / "experiment_summary.txt"

    lines = []
    lines.append("Preliminary visual navigation experiment summary")
    lines.append("=" * 55)
    lines.append("")
    lines.append(f"SRT file: {args.srt}")
    lines.append(f"Video file: {args.video}")
    lines.append(f"Number of output telemetry samples: {len(projected)}")
    lines.append(f"Minimum altitude filter: {args.min_altitude} m")
    lines.append(f"Telemetry step: {args.telemetry_step if args.telemetry_step is not None else args.frame_step}")
    lines.append(f"Heading from GNSS: {args.heading_from_gnss}")
    lines.append(f"Heading smoothing window: {args.smooth_heading_window}")
    lines.append(f"Camera angle override: {args.camera_angle}")
    lines.append("")
    lines.append("Projection statistics:")
    lines.append(str(projected[[
        "used_altitude_m",
        "used_heading_deg",
        "camera_depression_deg",
        "center_offset_m",
    ]].describe()))
    lines.append("")

    summary_path.write_text("\n".join(lines), encoding="utf-8")


def run(args: argparse.Namespace) -> None:
    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    telemetry = parse_srt(args.srt)
    if telemetry.empty:
        raise RuntimeError("No telemetry records were parsed. Check the SRT format or file path.")

    prepared = prepare_telemetry_for_experiment(telemetry, args)
    projected = add_clean_center_projection(prepared, args)

    projected_csv = outdir / "projected_camera_center_path.csv"
    projected.to_csv(projected_csv, index=False)

    plot_png = outdir / "path_comparison.png"
    plot_paths(
        projected,
        plot_png,
        title="Clean drone path vs camera-center ground path",
    )

    kml_path = outdir / "path_comparison.kml"
    save_kml(projected, kml_path)

    write_experiment_summary(projected, outdir, args)

    print(f"Parsed telemetry records: {len(telemetry)}")
    print(f"Output telemetry samples: {len(projected)}")
    print(f"CSV: {projected_csv}")
    print(f"Plot: {plot_png}")
    print(f"KML: {kml_path}")
    print(f"Summary: {outdir / 'experiment_summary.txt'}")

    if args.video:
        frames_dir = outdir / "frames"
        frames = extract_frames(
            args.video,
            frames_dir,
            frame_step=args.frame_step,
            max_frames=args.max_frames,
        )

        frames_csv = outdir / "extracted_frames.csv"
        frames.to_csv(frames_csv, index=False)
        print(f"Extracted frames: {len(frames)} -> {frames_csv}")

        # Approximate synchronization by nearest timestamp.
        synced = pd.merge_asof(
            frames.sort_values("timestamp_s"),
            projected.sort_values("timestamp_s"),
            on="timestamp_s",
            direction="nearest",
            tolerance=1.0,
        )

        synced_csv = outdir / "frames_with_projected_coords.csv"
        synced.to_csv(synced_csv, index=False)
        print(f"Frame-coordinate table: {synced_csv}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run the clean geometry-only preliminary experiment.")

    parser.add_argument("--srt", required=True, help="Input DJI/Autel SRT telemetry file")
    parser.add_argument("--video", default=None, help="Optional input video file")
    parser.add_argument("--outdir", default="outputs/preliminary", help="Output directory")

    parser.add_argument("--frame-step", type=int, default=30, help="Extract one video frame every N frames")
    parser.add_argument(
        "--telemetry-step",
        type=int,
        default=None,
        help="Use one telemetry row every N rows. If omitted, uses --frame-step.",
    )
    parser.add_argument("--max-frames", type=int, default=None, help="Optional maximum number of frames to extract")

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
        help=(
            "Manual camera depression angle below the horizon, in degrees. "
            "Example: --camera-angle 60. If omitted, use SRT gimbal pitch or fallback pitch."
        ),
    )

    parser.add_argument(
        "--heading-from-gnss",
        action="store_true",
        help="Estimate heading from GNSS trajectory instead of relying on SRT yaw/heading.",
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
        help="Rolling circular smoothing window for heading.",
    )

    parser.add_argument(
        "--fallback-pitch-deg",
        type=float,
        default=-60.0,
        help="Used if SRT lacks gimbal pitch and --camera-angle is not provided.",
    )
    parser.add_argument(
        "--fallback-altitude-m",
        type=float,
        default=119.0,
        help="Used only if SRT lacks altitude.",
    )
    parser.add_argument(
        "--pitch-convention",
        default="dji",
        choices=["dji", "positive_down"],
        help="Pitch convention for SRT gimbal pitch.",
    )

    run(parser.parse_args())