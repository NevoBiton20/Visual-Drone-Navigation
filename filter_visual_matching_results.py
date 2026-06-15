import pandas as pd
from pathlib import Path

INPUT_CSV = "outputs/v14_visual_matching/visual_localization_results.csv"
OUTPUT_CSV = "outputs/v14_visual_matching/visual_localization_results_filtered_inliers10.csv"
THRESHOLD = 10

df = pd.read_csv(INPUT_CSV)

filtered = df[df["homography_inliers"] >= THRESHOLD].copy()

Path(OUTPUT_CSV).parent.mkdir(parents=True, exist_ok=True)
filtered.to_csv(OUTPUT_CSV, index=False)

print(f"Input frames: {len(df)}")
print(f"Filtered frames: {len(filtered)}")
print(f"Kept: {100 * len(filtered) / len(df):.1f}%")
print()
print("Filtered error statistics:")
print(filtered["center_error_m"].describe())
print()
print(f"Saved filtered results to: {OUTPUT_CSV}")