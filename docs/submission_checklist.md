# Submission Checklist

## Required by Assignment

- [x] Literature review draft on visual navigation algorithms and tools.
- [x] Focus on low-flying drones, approximately 20-200 meters.
- [x] Focus on recent research and open-source implementations.
- [x] Complete preprocessing-stage algorithm.
- [x] Complete navigation-stage algorithm.
- [x] Suitable platform recommendation.
- [x] Editable code baseline for the provided videos and SRT files.
- [x] Preliminary experiment script for SRT-based path projection.

## Still Needed After Receiving Actual Data

- [ ] Put assignment videos/SRT files in `data/raw/`.
- [ ] Update `data/raw/pairs_template.csv` with exact filenames.
- [ ] Run `preliminary_experiment.py` on one selected testing video.
- [ ] Add generated `path_comparison.png` to the final report.
- [ ] Open generated `path_comparison.kml` in Google Earth and inspect result.
- [ ] Run `build_reference_database.py` on training videos.
- [ ] Run `localize_by_visual_matching.py` on a test video.
- [ ] Fill the Results section with actual numbers and screenshots.

## Recommended Extra Credit / Stronger Version

- [ ] Replace ORB with LightGlue/SuperPoint.
- [ ] Add DEM raycasting instead of flat-ground projection.
- [ ] Add semantic segmentation masks for roads/buildings/trees.
- [ ] Add Kalman filter smoothing.
- [ ] Evaluate localization error in meters.
