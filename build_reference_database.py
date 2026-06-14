"""Build the preprocessing/reference database required by the assignment.

For each training video + SRT pair, the script extracts sampled frames,
parses telemetry, computes camera-center ground coordinates, and writes a
reference CSV that can later be searched during GNSS-denied navigation.

Expected input CSV format for --pairs:
    video_path,srt_path,name
    data/raw/v11.mp4,data/raw/v11.srt,v11
    data/raw/v12.mp4,data/raw/v12.srt,v12
"""

from __future__ import annotations

import argparse
from pathlib import Path

import pandas as pd

from src.projection_utils import add_center_ground_projection
from src.srt_parser import parse_srt
from src.video_utils import extract_frames


def process_pair(row: pd.Series, out_root: Path, args: argparse.Namespace) -> pd.DataFrame:
    name = row.get("name") or Path(row["video_path"]).stem
    pair_dir = out_root / str(name)
    pair_dir.mkdir(parents=True, exist_ok=True)

    frames = extract_frames(row["video_path"], pair_dir / "frames", frame_step=args.frame_step, max_frames=args.max_frames)
    telemetry = parse_srt(row["srt_path"])
    projected = add_center_ground_projection(
        telemetry,
        fallback_pitch_deg=args.fallback_pitch_deg,
        fallback_altitude_m=args.fallback_altitude_m,
        pitch_convention=args.pitch_convention,
    )

    if frames.empty:
        raise RuntimeError(f"No frames extracted for {row['video_path']}")
    if projected.empty:
        raise RuntimeError(f"No telemetry parsed for {row['srt_path']}")

    synced = pd.merge_asof(
        frames.sort_values("timestamp_s"),
        projected.sort_values("timestamp_s"),
        on="timestamp_s",
        direction="nearest",
        tolerance=1.0,
    )
    synced["source_name"] = name
    synced.to_csv(pair_dir / "reference_frames.csv", index=False)
    return synced


def main(args: argparse.Namespace) -> None:
    out_root = Path(args.outdir)
    out_root.mkdir(parents=True, exist_ok=True)

    pairs = pd.read_csv(args.pairs)
    all_rows = []
    for _, row in pairs.iterrows():
        print(f"Processing {row.to_dict()}")
        all_rows.append(process_pair(row, out_root, args))

    reference_db = pd.concat(all_rows, ignore_index=True)
    reference_db.to_csv(out_root / "reference_database.csv", index=False)
    print(f"Reference database written to {out_root / 'reference_database.csv'}")
    print(f"Rows: {len(reference_db)}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Build visual navigation reference database")
    parser.add_argument("--pairs", required=True, help="CSV with columns video_path,srt_path,name")
    parser.add_argument("--outdir", default="data/processed/reference_database")
    parser.add_argument("--frame-step", type=int, default=30)
    parser.add_argument("--max-frames", type=int, default=None)
    parser.add_argument("--fallback-pitch-deg", type=float, default=-60.0)
    parser.add_argument("--fallback-altitude-m", type=float, default=119.0)
    parser.add_argument("--pitch-convention", default="dji", choices=["dji", "positive_down"])
    main(parser.parse_args())
