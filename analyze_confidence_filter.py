import pandas as pd

RESULTS_CSV = "outputs/v14_visual_matching/visual_localization_results.csv"

df = pd.read_csv(RESULTS_CSV)

print("Total frames:", len(df))
print()

for threshold in [5, 8, 10, 12, 15, 20, 30]:
    filtered = df[df["homography_inliers"] >= threshold]

    print("=" * 60)
    print(f"Threshold: homography_inliers >= {threshold}")
    print(f"Kept frames: {len(filtered)} / {len(df)} = {100 * len(filtered) / len(df):.1f}%")

    if len(filtered) == 0:
        print("No frames kept.")
        continue

    stats = filtered["center_error_m"].describe()

    print("Error statistics [meters]:")
    print(f"Mean:   {stats['mean']:.2f}")
    print(f"Median: {stats['50%']:.2f}")
    print(f"75%:    {stats['75%']:.2f}")
    print(f"Max:    {stats['max']:.2f}")

print("=" * 60)