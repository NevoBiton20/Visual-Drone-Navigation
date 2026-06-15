# Visual Navigation for Drones / Ex1 Project Package

This package contains a complete starter implementation for the university assignment:

- Literature review draft: `docs/report_draft.md`
- Preprocessing/reference database builder: `build_reference_database.py`
- Preliminary geometry experiment: `preliminary_experiment.py`
- GNSS-denied visual matching baseline: `localize_by_visual_matching.py`
- Reusable modules in `src/`

## 1. Install

```bash
python -m venv .venv
.venv\Scripts\activate        # Windows
# source .venv/bin/activate    # Linux/Mac
pip install -r requirements.txt
```

## 2. Place data

Put videos and SRT files in:

```text
data/raw/
```

Example:

```text
data/raw/v11.mp4
data/raw/v11.srt
data/raw/v12.mp4
data/raw/v12.srt
```

Edit:

```text
data/raw/pairs_template.csv
```

so it points to the actual filenames.

## 3. Run preliminary experiment

This computes the camera-center ground path from SRT telemetry and saves CSV, PNG, and KML outputs.

```bash
python preliminary_experiment.py --srt data/raw/v11.srt --video data/raw/v11.mp4 --outdir outputs/v11 --frame-step 30
```

Output:

```text
outputs/v11/projected_camera_center_path.csv
outputs/v11/path_comparison.png
outputs/v11/path_comparison.kml
outputs/v11/frames_with_projected_coords.csv
```

Open the KML file in Google Earth to inspect the drone path and the projected camera-center path.

## 4. Build preprocessing/reference database

```bash
python build_reference_database.py --pairs data/raw/pairs_template.csv --outdir data/processed/reference_database --frame-step 30
```

Output:

```text
data/processed/reference_database/reference_database.csv
```

This CSV contains sampled frames, telemetry, and projected camera-center coordinates.

## 5. Run GNSS-denied visual matching baseline

Use a video as if its GNSS is hidden. The script estimates location by matching its frames against the reference database.

```bash
python localize_by_visual_matching.py \
  --reference-db data/processed/reference_database/reference_database.csv \
  --query-video data/raw/test_video.mp4 \
  --outdir outputs/visual_matching \
  --frame-step 30 \
  --max-frames 30
```

Output:

```text
outputs/visual_matching/visual_localization_results.csv
```

## 6. Baseline assumptions

- Flat ground.
- DJI pitch convention: `0° = horizon`, `-90° = straight down`.
- If pitch or altitude is missing in the SRT, fallback values are used.
- ORB is the default matching method.

## 7. Recommended improvements

For a stronger final project:

1. Replace flat-ground projection with DEM/terrain raycasting.
2. Replace ORB matching with SuperPoint + LightGlue.
3. Use semantic segmentation to ignore unstable objects and emphasize roads/buildings.
4. Use a Kalman filter or particle filter for temporal smoothing.
5. Use an orthomosaic or satellite map instead of raw reference frames.

## 8. Suggested final submission structure

```text
1. Introduction
2. Literature Review
3. Problem Definition
4. Proposed Preprocessing Algorithm
5. Proposed Real-Time Navigation Algorithm
6. Implementation Platform
7. Preliminary Experiment
8. Results
9. Limitations
10. Future Work
```
