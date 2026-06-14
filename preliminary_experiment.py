"""Preliminary experiment for Ex1: geometry-only camera-center projection.

Usage examples:
    python preliminary_experiment.py --srt data/raw/v11.srt --outdir outputs/v11
    python preliminary_experiment.py --srt data/raw/v11.srt --video data/raw/v11.mp4 --outdir outputs/v11 --frame-step 30

The script:
1. Parses the drone SRT telemetry.
2. Computes the ground coordinate seen at the center of the image using height,
   heading, and camera/gimbal pitch.
3. Exports CSV, PNG plot, and KML file for Google Earth.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import pandas as pd

from src.projection_utils import add_center_ground_projection
from src.srt_parser import parse_srt
from src.video_utils import extract_frames
from src.plotting import plot_paths, save_kml


def run(args: argparse.Namespace) -> None:
    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    telemetry = parse_srt(args.srt)
    if telemetry.empty:
        raise RuntimeError("No telemetry records were parsed. Check the SRT format or file path.")

    projected = add_center_ground_projection(
        telemetry,
        fallback_pitch_deg=args.fallback_pitch_deg,
        fallback_altitude_m=args.fallback_altitude_m,
        pitch_convention=args.pitch_convention,
    )

    projected_csv = outdir / "projected_camera_center_path.csv"
    projected.to_csv(projected_csv, index=False)

    plot_png = outdir / "path_comparison.png"
    plot_paths(projected, plot_png)

    kml_path = outdir / "path_comparison.kml"
    save_kml(projected, kml_path)

    print(f"Telemetry records: {len(projected)}")
    print(f"CSV: {projected_csv}")
    print(f"Plot: {plot_png}")
    print(f"KML: {kml_path}")

    if args.video:
        frames_dir = outdir / "frames"
        frames = extract_frames(args.video, frames_dir, frame_step=args.frame_step, max_frames=args.max_frames)
        frames_csv = outdir / "extracted_frames.csv"
        frames.to_csv(frames_csv, index=False)
        print(f"Extracted frames: {len(frames)} -> {frames_csv}")

        # Optional approximate synchronization by nearest timestamp.
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
    parser = argparse.ArgumentParser(description="Run the geometry-only preliminary experiment.")
    parser.add_argument("--srt", required=True, help="Input DJI/Autel SRT telemetry file")
    parser.add_argument("--video", default=None, help="Optional input video file")
    parser.add_argument("--outdir", default="outputs/preliminary", help="Output directory")
    parser.add_argument("--frame-step", type=int, default=30, help="Extract one frame every N frames")
    parser.add_argument("--max-frames", type=int, default=None, help="Optional maximum number of frames to extract")
    parser.add_argument("--fallback-pitch-deg", type=float, default=-60.0, help="Used if SRT lacks gimbal pitch")
    parser.add_argument("--fallback-altitude-m", type=float, default=119.0, help="Used if SRT lacks altitude")
    parser.add_argument("--pitch-convention", default="dji", choices=["dji", "positive_down"], help="Pitch convention")
    run(parser.parse_args())
