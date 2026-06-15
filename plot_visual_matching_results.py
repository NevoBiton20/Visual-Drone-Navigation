"""Plot evaluation results for the GNSS-denied visual matching baseline."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


def plot_estimated_vs_ground_truth(df: pd.DataFrame, outdir: Path) -> None:
    plt.figure(figsize=(10, 8))

    valid_gt = df.dropna(subset=["gt_center_latitude", "gt_center_longitude"])
    valid_est = df.dropna(subset=["estimated_center_latitude", "estimated_center_longitude"])

    plt.plot(
        valid_gt["gt_center_longitude"],
        valid_gt["gt_center_latitude"],
        marker="o",
        markersize=3,
        linewidth=1,
        label="Ground-truth camera-center path",
    )

    plt.plot(
        valid_est["estimated_center_longitude"],
        valid_est["estimated_center_latitude"],
        marker="o",
        markersize=3,
        linewidth=1,
        label="Estimated visual localization path",
    )

    plt.title("GNSS-denied visual localization: estimated path vs ground truth")
    plt.xlabel("Longitude")
    plt.ylabel("Latitude")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(outdir / "estimated_vs_ground_truth_path.png", dpi=200)
    plt.close()


def plot_error_over_time(df: pd.DataFrame, outdir: Path) -> None:
    valid = df.dropna(subset=["center_error_m"])

    plt.figure(figsize=(12, 6))
    plt.plot(
        valid["query_timestamp_s"],
        valid["center_error_m"],
        marker="o",
        markersize=3,
        linewidth=1,
    )

    plt.title("Visual localization error over time")
    plt.xlabel("Query timestamp [s]")
    plt.ylabel("Camera-center localization error [m]")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(outdir / "localization_error_over_time.png", dpi=200)
    plt.close()


def plot_error_histogram(df: pd.DataFrame, outdir: Path) -> None:
    valid = df.dropna(subset=["center_error_m"])

    plt.figure(figsize=(10, 6))
    plt.hist(valid["center_error_m"], bins=25)

    plt.title("Distribution of visual localization errors")
    plt.xlabel("Camera-center localization error [m]")
    plt.ylabel("Number of query frames")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(outdir / "localization_error_histogram.png", dpi=200)
    plt.close()


def plot_error_by_source(df: pd.DataFrame, outdir: Path) -> None:
    valid = df.dropna(subset=["center_error_m", "best_source_name"])

    grouped = [
        group["center_error_m"].values
        for _, group in valid.groupby("best_source_name")
    ]

    labels = [
        name
        for name, _ in valid.groupby("best_source_name")
    ]

    plt.figure(figsize=(8, 6))
    plt.boxplot(grouped, tick_labels=labels)

    plt.title("Localization error by matched reference video")
    plt.xlabel("Matched reference video")
    plt.ylabel("Camera-center localization error [m]")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(outdir / "localization_error_by_source.png", dpi=200)
    plt.close()


def write_report_numbers(df: pd.DataFrame, outdir: Path) -> None:
    valid = df.dropna(subset=["center_error_m"])

    lines = []
    lines.append("Visual localization evaluation numbers")
    lines.append("=" * 50)
    lines.append("")
    lines.append(f"Total query frames: {len(df)}")
    lines.append(f"Frames with valid error: {len(valid)}")
    lines.append("")
    lines.append("Error statistics [meters]:")
    lines.append(str(valid["center_error_m"].describe()))
    lines.append("")
    lines.append("Matched source counts:")
    lines.append(str(df["best_source_name"].value_counts()))
    lines.append("")
    lines.append("Error by matched source [meters]:")
    lines.append(str(valid.groupby("best_source_name")["center_error_m"].describe()))
    lines.append("")
    lines.append("Best 10 matches:")
    lines.append(str(valid.sort_values("center_error_m")[
        [
            "query_frame_index",
            "best_source_name",
            "match_score",
            "good_matches",
            "homography_inliers",
            "center_error_m",
        ]
    ].head(10)))
    lines.append("")
    lines.append("Worst 10 matches:")
    lines.append(str(valid.sort_values("center_error_m", ascending=False)[
        [
            "query_frame_index",
            "best_source_name",
            "match_score",
            "good_matches",
            "homography_inliers",
            "center_error_m",
        ]
    ].head(10)))

    (outdir / "visual_matching_report_numbers.txt").write_text(
        "\n".join(lines),
        encoding="utf-8",
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--results", required=True)
    parser.add_argument("--outdir", required=True)

    args = parser.parse_args()

    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    df = pd.read_csv(args.results)

    plot_estimated_vs_ground_truth(df, outdir)
    plot_error_over_time(df, outdir)
    plot_error_histogram(df, outdir)
    plot_error_by_source(df, outdir)
    write_report_numbers(df, outdir)

    print("Saved plots to:", outdir)


if __name__ == "__main__":
    main()