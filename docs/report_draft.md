# Ex1 — Visual Navigation for Drones in GNSS-Denied Conditions

## 1. Introduction

This assignment studies visual navigation for low-flying drones in GNSS-denied conditions. The practical problem is: given a drone video stream and telemetry without GNSS, estimate the geographic coordinate of the ground point observed at the center of the image. The available training data contains videos with SRT telemetry, including GNSS position, barometric/relative height, and camera/gimbal orientation. Therefore, the training flights can be used to build a geo-referenced visual reference database. During a later flight without GNSS, the system localizes the incoming video frames by matching them against the reference database and computing the ground coordinate of the current camera center.

The proposed solution combines three components:

1. Camera geometry and telemetry-based ground projection.
2. Visual image matching against geo-referenced reference frames or an orthomosaic.
3. Temporal smoothing to produce a stable real-time navigation output.

## 2. Literature Review

### 2.1 Visual odometry, visual-inertial odometry, and SLAM

Visual odometry estimates the camera motion between consecutive frames. Visual-inertial odometry also uses inertial measurements, and SLAM jointly estimates camera pose and builds a map. These methods are useful for short-term navigation and hovering because they estimate relative motion even without GNSS. However, they usually accumulate drift over time unless they are corrected by loop closure, map matching, or another absolute reference. ORB-SLAM3 is a strong open-source baseline for visual and visual-inertial SLAM, supporting monocular, stereo, RGB-D, and inertial configurations. It is suitable for estimating the relative trajectory of the drone, but by itself it does not directly solve the assignment's requirement of producing geographic coordinates unless the map is geo-referenced.

### 2.2 Absolute visual localization using maps

Absolute visual localization aims to estimate the global position of the drone by matching the current aerial image to a geo-referenced map, orthophoto, or satellite image. This is closer to the assignment because the output is a coordinate. UAV-VisLoc defines the UAV localization problem as matching UAV imagery to an orthorectified satellite map, where coordinates are available for map pixels. The dataset includes UAV images with location, height, heading, and other metadata, which makes it highly relevant for designing the proposed pipeline.

### 2.3 GNSS-free localization with satellite imagery

Several recent methods address GNSS-free UAV localization by matching onboard drone imagery with satellite or aerial maps. A practical pipeline usually contains a coarse image retrieval stage followed by a fine registration stage. Coarse retrieval finds candidate map tiles or reference frames; fine registration estimates a geometric transformation between the drone image and the selected map/reference image. The 2025 hierarchical absolute localization system for low-altitude UAVs follows this logic: it combines image retrieval and image registration, and also uses inertial correction and local map updates.

### 2.4 Feature matching methods

Classical feature matching methods such as SIFT, ORB, and AKAZE are simple and interpretable. ORB is fast and available in OpenCV, so it is a good first baseline. However, drone images often contain viewpoint change, scale change, motion blur, lighting differences, and seasonal differences. Modern learned methods such as SuperGlue and LightGlue improve matching by using neural networks to reason about correspondences. LightGlue is particularly useful for this assignment because it is designed for fast local feature matching and has open-source code.

### 2.5 Drone image geolocation by raycasting

A separate but highly relevant direction is metadata-based pixel geolocation. OpenAthena geolocates a selected pixel in drone imagery by combining camera metadata with a terrain elevation model. This matches the assignment's camera-center-coordinate requirement very closely. In this project, the first baseline uses a simpler flat-ground version: from altitude and camera pitch, it computes the ground intersection of the center camera ray. A stronger version would replace flat-ground projection with DEM raycasting.

### 2.6 Semantic segmentation for UAV images

Semantic segmentation can detect roads, buildings, trees, cars, humans, and other objects. Segmentation is useful as supporting information: it can reject bad visual matches, emphasize stable classes such as roads/buildings, and ignore unstable classes such as cars or pedestrians. However, segmentation alone does not produce geographic coordinates. SegFormer is a suitable recent family of segmentation models for UAV imagery, with smaller variants for real-time use and larger variants for higher accuracy.

## 3. Problem Definition

Input during preprocessing:

- Drone videos.
- SRT telemetry containing GNSS, altitude/barometric height, heading/yaw, and gimbal pitch.

Input during GNSS-denied navigation:

- Real-time video stream.
- Telemetry without GNSS, for example height, camera angle, and possibly heading/IMU.

Output:

- Estimated geographic coordinate of the ground point seen at the center of the video frame.

Assumptions for the first baseline:

- The ground is locally flat.
- The camera center ray is enough for the preliminary experiment.
- Gimbal pitch follows the common DJI convention: 0° is horizon and -90° is nadir.
- If SRT lacks pitch or altitude, assignment-provided constants are used.

## 4. Proposed Preprocessing Algorithm

For each training video:

1. Extract sampled frames, for example one frame per second.
2. Parse SRT telemetry.
3. Synchronize each extracted frame with the nearest telemetry timestamp.
4. Compute the camera-center ground coordinate using altitude, heading, and camera pitch.
5. Store frame path, timestamp, drone GNSS coordinate, camera-center coordinate, altitude, pitch, heading, and source video name.
6. Extract visual descriptors from each reference frame.
7. Save a reference database for real-time localization.

### 4.1 Camera-center projection

Let:

- `h` be drone height above local ground.
- `theta` be the camera depression angle below the horizon.
- `psi` be drone/camera heading.

The horizontal distance from the drone to the center ground point is:

```text
d = h / tan(theta)
```

Then the ground center coordinate is computed by moving from the drone GNSS coordinate by distance `d` in bearing direction `psi`.

For example:

- Height 119 m, camera depression 60°: `d ≈ 68.7 m`.
- Height 120 m, camera depression 45°: `d ≈ 120 m`.
- Height 50 m, camera depression 45°: `d ≈ 50 m`.

## 5. Proposed Real-Time Navigation Algorithm

For each incoming GNSS-denied frame:

1. Extract features from the current frame.
2. Retrieve visually similar reference frames or map tiles from the preprocessing database.
3. Match the query frame with candidate reference frames using ORB in the baseline, and LightGlue/SuperPoint in the improved version.
4. Estimate a homography with RANSAC.
5. Use the best inlier score to choose the best reference frame.
6. Use the matched reference frame's stored camera-center coordinate as the first estimate.
7. Optionally refine the estimate using the homography pixel shift and the estimated ground sampling distance.
8. Apply temporal smoothing with a moving average or Kalman filter.
9. Output the estimated camera-center coordinate in real time.

## 6. Platform Choice

The recommended implementation platform is Python with OpenCV and pandas for the baseline, because the assignment requires preprocessing video/SRT files and performing a preliminary experiment. ORB matching is used in the baseline because it is simple, fast, and easy to explain. For the advanced version, the matching module can be replaced with SuperPoint + LightGlue. For pixel-level geolocation using terrain, OpenAthena is the closest conceptual reference, but the implementation should begin with a flat-ground version and later add DEM raycasting.

The platform decision is therefore:

- Baseline: Python + OpenCV + SRT parser + flat-ground camera ray projection.
- Improved matching: SuperPoint + LightGlue.
- Optional mapping: OpenREALM-style orthomosaic/reference map.
- Optional semantic support: SegFormer for roads/buildings/trees and match filtering.

## 7. Preliminary Experiment

The preliminary experiment uses a test video and its SRT telemetry:

1. Parse SRT file.
2. For each telemetry record, read latitude, longitude, height, heading, and gimbal pitch.
3. Compute the projected ground coordinate of the center pixel.
4. Export a CSV file with both the drone GNSS coordinate and the projected camera-center coordinate.
5. Plot both paths.
6. Export a KML file so the paths can be opened in Google Earth.

This experiment validates the geometric part of the system. It also provides a sanity check: when the camera points straight down, the center path should be close to the drone GNSS path; when the camera is oblique, the center path should be shifted in the heading direction.

## 8. Expected Limitations

The baseline has several limitations:

- Flat-ground projection ignores terrain elevation.
- SRT metadata may differ between drone models.
- Camera FOV and gimbal pitch may not be perfectly calibrated.
- ORB matching may fail under strong viewpoint, lighting, or altitude differences.
- Moving cars, people, trees, and shadows may create unstable features.
- Pure visual matching can confuse repetitive roads/buildings.

## 9. Suggested Improvements

1. Replace flat ground with DEM raycasting.
2. Replace ORB with SuperPoint + LightGlue.
3. Use semantic segmentation to prefer stable classes such as roads and buildings.
4. Build an orthomosaic from the training videos.
5. Use a Kalman filter or particle filter for temporal smoothing.
6. Use heading/IMU to restrict the search region and reduce false matches.
7. Evaluate performance with coordinate error in meters.

## 10. Evaluation Plan

Possible metrics:

- Localization error in meters between estimated and ground-truth GNSS camera/drone coordinate.
- Match inlier count.
- Percentage of frames successfully localized.
- Runtime per frame.
- Robustness by altitude, lighting, drone model, and camera angle.

For videos with GNSS available, a GNSS-denied simulation can be created by hiding the GNSS column during localization and using it only for evaluation.

## 11. References / Tools

- ORB-SLAM3: visual, visual-inertial, and multi-map SLAM.
- UAV-VisLoc: UAV visual localization with satellite map matching.
- Vision-based GNSS-Free Localization for UAVs in the Wild.
- OpenREALM: real-time UAV mapping and orthophoto generation.
- OpenAthena: drone image pixel geolocation using metadata and terrain.
- LightGlue: fast learned local feature matching.
- SuperGlue: learned feature matching with graph neural networks.
- SegFormer for semantic segmentation of UAV images.
