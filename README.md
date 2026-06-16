## Run the real-time visual navigation demo

This demo treats `v14.mp4` as a GNSS-denied query video and estimates the camera-center ground route by matching each frame against a geo-referenced database built from previous drone videos and GIS tiles.

### Required data layout

Place the videos and SRT files here:

```text
data/raw/v11.mp4
data/raw/v11.srt
data/raw/v12.mp4
data/raw/v12.srt
data/raw/v13.mp4
data/raw/v13.srt
data/raw/v14.mp4
data/raw/v14.srt
```

The GIS split-tile CSV should be:

```text
data/gis/tiles_split.csv
```

### 1. Build the previous-video reference database

```bat
python build_reference_database.py ^
  --pairs data/raw/pairs_train_leave_v14_out.csv ^
  --outdir data/processed/reference_train_v11_v12_v13 ^
  --frame-step 30 ^
  --min-altitude 20 ^
  --camera-angle 60 ^
  --heading-from-gnss ^
  --smooth-heading-window 5
```

### 2. Build the GIS tile database

```bat
python build_gis_tile_database.py ^
  --tiles data/gis/tiles_split.csv ^
  --outdir data/processed/gis_tiles_split
```

### 3. Run the hybrid real-time visual navigation algorithm

```bat
python realtime_visual_navigation_gis.py ^
  --video-reference-db data/processed/reference_train_v11_v12_v13/reference_database.csv ^
  --gis-reference-db data/processed/gis_tiles_split/gis_reference_database.csv ^
  --query-video data/raw/v14.mp4 ^
  --query-srt data/raw/v14.srt ^
  --outdir outputs/v14_realtime_gis_split ^
  --frame-step 60 ^
  --camera-angle 60 ^
  --min-altitude 20 ^
  --min-inliers 10
```

Main outputs:

```text
outputs/v14_realtime_gis_split/realtime_navigation_results.csv
outputs/v14_realtime_gis_split/realtime_navigation_summary.txt
```

### 4. Export the result route to KML

```bat
python export_realtime_results_to_kml.py ^
  --results outputs/v14_realtime_gis_split/realtime_navigation_results.csv ^
  --out outputs/v14_realtime_gis_split/realtime_navigation_result_no_points.kml
```

Open this file in Google Earth:

```text
outputs/v14_realtime_gis_split/realtime_navigation_result_no_points.kml
```

KML colors:

```text
Blue   = ground-truth camera-center path from v14 SRT, used only for evaluation
Orange = accepted estimated visual-navigation path
Gray   = all raw estimated visual-navigation outputs
```

### Method summary

The method is geo-referenced reference-based visual localization. In the offline stage, previous drone videos and GIS tiles are converted into a visual reference database with known coordinates. In the navigation stage, each query frame is matched to the database using ORB feature extraction, descriptor matching, and RANSAC homography verification. Reliable matches are accepted using a minimum homography-inlier threshold, and the accepted coordinates are exported as the estimated route.
