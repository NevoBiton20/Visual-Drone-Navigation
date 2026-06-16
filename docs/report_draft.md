# Visual Navigation for Drones

## GNSS-Denied Optical Navigation Using Drone Video and SRT Telemetry

### 1. Introduction

The goal of this assignment is to study visual navigation for drones in GNSS-denied conditions. In normal drone operation, the drone can rely on GNSS measurements such as GPS coordinates in order to know its position. However, in many real-world scenarios GNSS may be unavailable, unreliable, jammed, spoofed, or blocked. In such cases, the drone must estimate its position using alternative sensors, mainly the onboard camera and additional telemetry such as altitude, camera angle, and heading.

This project focuses on a practical optical navigation problem. Given drone flight data that includes video and SRT telemetry with GNSS information, the system first preprocesses the data into a geo-referenced visual database. Later, given a new video stream without GNSS, the system estimates the geographic coordinate of the ground point observed at the center of the video frame.

The implemented system includes three main parts:

1. A geometry-based preliminary experiment that projects the camera-center ray onto the ground using SRT telemetry.
2. A preprocessing stage that builds a geo-referenced database from GNSS-labeled flights.
3. A GNSS-denied visual localization stage that matches frames from a query video to the reference database.

In addition, a confidence-filtering mechanism is added in order to reject unreliable visual matches.

---

### 2. Problem Definition

The assignment considers a drone equipped with a video camera. During the preprocessing stage, the drone flight includes GNSS information. During the navigation stage, the drone receives a new video stream and telemetry, but without GNSS positioning.

The required output is not necessarily the exact body position of the drone. Instead, the main required output is the coordinate of the point on the ground that appears at the center of the video frame. This point is called in this report the camera-center ground point.

For every frame, the system aims to estimate:

[
P_c = (\text{latitude}_c, \text{longitude}_c)
]

where (P_c) is the geographic coordinate of the ground point observed at the center of the image.

The available data includes:

* Drone video.
* SRT telemetry.
* GNSS latitude and longitude during preprocessing.
* Relative altitude from the SRT file.
* Camera angle from the dataset description.
* Frame timestamps.
* In some cases, missing yaw/gimbal fields.

In the used SRT files, the GNSS coordinates and relative altitude were available, but explicit yaw and gimbal pitch fields were not consistently available. Therefore, the camera angle was taken from the assignment description, and the heading was estimated from the GNSS trajectory.

---

### 3. Literature Review

Visual navigation for drones can be approached using several families of algorithms.

#### 3.1 Visual Odometry and Visual-Inertial Odometry

Visual Odometry estimates the motion of a camera by tracking visual features across consecutive frames. Visual-Inertial Odometry extends this approach by also using inertial measurements from an IMU. These methods are useful for estimating relative motion, especially over short time intervals.

The main weakness of pure visual odometry is drift. Small errors accumulate over time, so the estimated position becomes less accurate as the flight continues. Visual-Inertial Odometry reduces drift, but still usually requires loop closure, map matching, or external correction to obtain a globally accurate position.

#### 3.2 SLAM

Simultaneous Localization and Mapping, or SLAM, estimates both the camera trajectory and a map of the environment. ORB-SLAM3 is a well-known open-source SLAM system that supports monocular, stereo, RGB-D, visual-inertial, and multi-map SLAM. It is relevant because it demonstrates how feature-based methods and loop closure can support real-time camera localization.

However, SLAM alone does not directly solve the assignment’s geographic localization problem. SLAM gives a trajectory in a local coordinate frame unless it is connected to a geo-referenced map or initialized with global coordinates. Therefore, SLAM is useful as a component, but the assignment also requires georeferencing.

#### 3.3 UAV Mapping and Orthomosaic Construction

Another relevant direction is UAV mapping. OpenREALM is an open-source framework for real-time aerial mapping. It can create image mosaics and orthophotos from UAV video and georeference them using the UAV’s global position. This is close to the preprocessing stage required in the assignment, where GNSS-labeled video is converted into a reference database or visual map.

The main difference is that this project implements a lighter solution based on sampled frames and camera-center coordinates, instead of producing a full orthomosaic.

#### 3.4 Drone-to-Map Visual Localization

Drone-to-map localization directly matches a drone image to a geo-referenced map, satellite image, or previously built visual database. This direction is highly relevant to GNSS-denied navigation because it can provide absolute geographic coordinates rather than only relative motion.

UAV-VisLoc defines a similar task: given a drone image, estimate the UAV position by matching it to a satellite map. The dataset includes drone images, center coordinates, height, shooting date, and heading. This supports the idea that absolute UAV localization can be formulated as an image retrieval and image matching problem.

Another related direction is vision-based GNSS-free localization using open-source satellite imagery. In this approach, drone images are matched to geo-referenced map tiles, and the matched tile is used to estimate the drone’s location.

#### 3.5 Feature Matching

Classical feature matching methods such as ORB, SIFT, and AKAZE detect local image keypoints and match descriptors between images. ORB was selected in this project because it is fast, available in OpenCV, and suitable for a practical baseline.

Modern learned feature matching methods such as SuperGlue and LightGlue provide stronger matching performance. SuperGlue uses a graph neural network to match local features and reject non-matchable points. LightGlue improves efficiency and accuracy using adaptive computation. These methods are promising future replacements for ORB in this project.

#### 3.6 Semantic Segmentation

Semantic segmentation can identify classes such as roads, buildings, trees, cars, humans, and pedestrian crossings. In drone navigation, segmentation can help reject bad visual matches and focus localization on stable landmarks. SegFormer has been evaluated for UAV remote sensing image segmentation and is relevant for this purpose.

In this project, semantic segmentation was reviewed but not implemented. The implemented navigation baseline focuses on geometric projection and visual feature matching.

---

### 4. Proposed System Architecture

The proposed system has two main stages:

1. Preprocessing stage.
2. GNSS-denied navigation stage.

The preprocessing stage uses videos that include GNSS telemetry. It extracts frames, parses the SRT files, computes the camera-center ground coordinate for each frame, and stores the result in a reference database.

The navigation stage receives a new video stream without GNSS. For each query frame, it extracts visual features, matches the frame against the reference database, selects the best matching reference frame, and uses the matched reference frame’s known coordinate as the estimated location.

The complete system is:

```
GNSS-labeled videos + SRT
        ↓
Frame extraction
        ↓
SRT parsing
        ↓
Camera-center ground projection
        ↓
Geo-referenced reference database
        ↓
GNSS-denied query video
        ↓
ORB feature extraction and matching
        ↓
Best reference frame selection
        ↓
Estimated camera-center coordinate
        ↓
Evaluation using query SRT ground truth
```

---

### 5. Camera-Center Ground Projection

The preliminary geometric model assumes locally flat ground. Given the drone altitude (h) and the camera depression angle (\theta), the horizontal ground offset from the drone to the camera-center ground point is:

[
d = \frac{h}{\tan(\theta)}
]

where:

* (h) is the relative altitude from the SRT file.
* (\theta) is the camera depression angle below the horizon.
* (d) is the distance in meters from the drone to the observed center point on the ground.

For the DJI Mini 3 Pro videos, the dataset description states that the camera angle is 60 degrees. Therefore:

[
d = \frac{h}{\tan(60^\circ)}
]

For example, at an altitude of approximately 100 meters:

[
d \approx \frac{100}{1.732} \approx 57.7 \text{ meters}
]

The SRT file provides the drone latitude and longitude. The heading is estimated from the GNSS trajectory because the parsed SRT metadata did not contain reliable yaw fields. The final camera-center coordinate is computed by moving (d) meters from the drone coordinate in the estimated heading direction.

---

### 6. Preprocessing Stage

The preprocessing stage builds the reference database.

For each video/SRT pair:

1. Extract one frame every fixed number of frames.
2. Parse the SRT telemetry.
3. Extract latitude, longitude, relative altitude, and timestamp.
4. Filter low-altitude frames.
5. Estimate heading from GNSS movement.
6. Smooth the heading using a circular rolling mean.
7. Compute the camera-center ground point.
8. Synchronize extracted frames with telemetry timestamps.
9. Save the results to a reference database.

The reference database contains, for each sampled frame:

* Frame image path.
* Timestamp.
* Drone latitude and longitude.
* Relative altitude.
* Estimated heading.
* Camera depression angle.
* Camera-center latitude and longitude.
* Source video name.

The leave-one-out preprocessing experiment used:

* Reference videos: `v11`, `v12`, `v13`.
* Query video left out for testing: `v14`.

The resulting leave-one-out reference database contained 541 reference frames:

| Source video | Number of reference frames |
| ------------ | -------------------------: |
| v11          |                        308 |
| v12          |                        115 |
| v13          |                        118 |
| Total        |                        541 |

The videos had different altitudes, which increased the variety of the reference database. The approximate average altitudes and corresponding camera-center offsets were:

| Source video | Mean altitude | Mean camera-center offset |
| ------------ | ------------: | ------------------------: |
| v11          |       96.86 m |                   55.92 m |
| v12          |       30.92 m |                   17.85 m |
| v13          |       49.79 m |                   28.75 m |

This confirms that the database contains examples from different flight heights.

---

### 7. GNSS-Denied Navigation Stage

The navigation stage treats `v14` as a GNSS-denied query video. The SRT of `v14` is not used for localization. It is used only later for evaluation.

For each query frame:

1. Extract the frame from `v14`.
2. Convert the frame to grayscale.
3. Extract ORB keypoints and descriptors.
4. Match the query descriptors against each reference frame.
5. Apply Lowe’s ratio test to keep stronger matches.
6. Estimate a homography using RANSAC.
7. Use the number of homography inliers as a confidence score.
8. Select the best reference frame.
9. Use the matched reference frame’s camera-center coordinate as the estimated position.

The first version always returns the best match, even if the match is weak. This is called the baseline method.

---

### 8. Preliminary Geometry Experiment

The first experiment tests only the geometric projection stage.

The input was one video and its corresponding SRT file. For each sampled frame, the system extracted the GNSS position and relative altitude from the SRT. The camera angle was set to 60 degrees according to the dataset description. The heading was estimated from GNSS movement.

The result compares:

* The real drone GNSS path.
* The projected camera-center ground path.

The projected path follows the general shape of the GNSS path but is shifted forward in the viewing direction. This is expected because the camera is tilted forward rather than pointing straight down.

The preliminary experiment shows that the SRT parser, altitude extraction, heading estimation, and camera-center projection work correctly.

**Figure:** `docs/figures/preliminary_path_comparison.png`

---

### 9. Visual Matching Experiment

The second experiment tests GNSS-denied visual localization.

The setup was:

* Reference database: `v11`, `v12`, `v13`.
* Query video: `v14`.
* Query SRT: used only for evaluation.
* Feature method: ORB.
* Matching method: descriptor matching with ratio test.
* Geometric verification: RANSAC homography.
* Localization estimate: camera-center coordinate of the best matched reference frame.

The system processed 126 query frames from `v14`.

The baseline results were:

| Metric                |    Value |
| --------------------- | -------: |
| Query frames          |      126 |
| Mean error            | 183.25 m |
| Median error          |  98.48 m |
| 75th percentile error | 294.01 m |
| Maximum error         | 817.67 m |

The matched reference source counts were:

| Matched source | Count |
| -------------- | ----: |
| v13            |    58 |
| v11            |    57 |
| v12            |    11 |

The best individual matches achieved very low error, including errors of approximately 2.7 m, 5.7 m, 10 m, and 11 m. This shows that the visual matching can correctly identify some locations. However, the baseline also produced large errors in some frames, especially when visually similar roads, buildings, or repeated textures were matched to geographically different places.

Visual inspection of the match examples showed that many feature matches align with real objects in the images. Therefore, the problem is not that feature matching completely fails. Instead, the limitation is that the baseline uses the coordinate of the matched reference frame directly and does not yet refine the position using the homography.

**Figures:**

* `docs/figures/visual_matching_path_comparison.png`
* `docs/figures/visual_matching_error_over_time.png`
* `docs/figures/visual_matching_error_histogram.png`
* `docs/figures/visual_matching_error_by_source.png`

---

### 10. Confidence-Filtered Improvement

The baseline always returns a location, even when the match is weak. For navigation, this is risky because a weak match can produce a large position error.

To improve reliability, a confidence filter was added. The confidence score is based on the number of RANSAC homography inliers. A match is accepted only if:

[
\text{homography inliers} \geq 10
]

The threshold was selected by testing several values:

| Threshold    | Frames kept | Mean error | Median error | 75th percentile | Max error |
| ------------ | ----------: | ---------: | -----------: | --------------: | --------: |
| Inliers ≥ 5  |   126 / 126 |   183.25 m |      98.48 m |        294.01 m |  817.67 m |
| Inliers ≥ 8  |    76 / 126 |   118.31 m |      52.28 m |        117.58 m |  538.45 m |
| Inliers ≥ 10 |    54 / 126 |    50.27 m |      48.28 m |         59.27 m |  150.33 m |
| Inliers ≥ 12 |    47 / 126 |    48.89 m |      45.90 m |         57.96 m |  150.33 m |
| Inliers ≥ 15 |    33 / 126 |    52.42 m |      46.44 m |         62.82 m |  126.31 m |
| Inliers ≥ 20 |    27 / 126 |    49.61 m |      45.32 m |         56.97 m |  126.31 m |
| Inliers ≥ 30 |    16 / 126 |    47.21 m |      45.32 m |         50.20 m |   80.50 m |

The threshold of 10 inliers was selected because it provides a good balance between accuracy and coverage. It keeps 54 out of 126 frames, which is 42.9% of the query frames, while significantly reducing the error.

The confidence-filtered method achieved:

| Method                            | Frames kept | Mean error | Median error | Max error |
| --------------------------------- | ----------: | ---------: | -----------: | --------: |
| ORB + homography baseline         |   126 / 126 |   183.25 m |      98.48 m |  817.67 m |
| Confidence-filtered, inliers ≥ 10 |    54 / 126 |    50.27 m |      48.28 m |  150.33 m |

This improvement shows that homography inlier count is an effective confidence measure for rejecting unreliable visual localizations.

**Figures:**

* `docs/figures/visual_matching_filtered_path_comparison.png`
* `docs/figures/visual_matching_filtered_error_over_time.png`
* `docs/figures/visual_matching_filtered_error_histogram.png`
* `docs/figures/visual_matching_filtered_error_by_source.png`

---

### 11. Discussion

The experiments show that a practical GNSS-denied visual localization pipeline can be built from video and SRT telemetry.

The preliminary geometry experiment validates the camera-center projection model. The projected path is shifted relative to the drone path in a way that matches the expected geometry of an angled camera.

The preprocessing stage successfully creates a geo-referenced reference database. This database stores visual frames and their estimated camera-center coordinates, allowing later query frames to be localized visually.

The visual matching baseline proves that local feature matching can identify real correspondences between query frames and reference frames. The match examples visually align well, which means the ORB feature matching and homography verification are meaningful.

However, the baseline also shows important limitations. A single best visual match is not always geographically correct. Similar visual structures may appear in different places, especially roads, fields, roofs, and repeated urban patterns. This can create large errors.

The confidence-filtered method improves the result by rejecting weak matches. This makes the system more reliable, although it reduces coverage because some frames are not localized.

---

### 12. Limitations

The current implementation has several limitations:

1. Flat-ground assumption
   The camera-center projection assumes locally flat ground. A digital elevation model would improve accuracy in hilly terrain.

2. Heading estimation from GNSS
   The SRT files did not provide reliable yaw or gimbal yaw fields. Therefore, heading was estimated from GNSS movement. This can be noisy during turns or slow movement.

3. No homography-based coordinate refinement
   The current visual localization method uses the coordinate of the matched reference frame directly. It does not yet use the homography to estimate the exact displacement between the query frame center and the reference frame center.

4. Classical ORB features
   ORB is fast and simple, but it is weaker than modern learned feature matchers such as SuperGlue or LightGlue.

5. No temporal filtering
   Each query frame is localized independently. A Kalman filter, particle filter, or sequence matching approach could smooth the estimated path.

6. No semantic filtering
   Semantic segmentation was not implemented. Roads, buildings, roundabouts, and other stable landmarks could help improve matching quality.

7. Limited dataset size
   The experiment used four videos, with three videos for the reference database and one video for testing. A larger dataset would provide stronger evaluation.

---

### Final Project Extension: Real-Time Visual Navigation Using Previous Videos and GIS Data

The final project extends Ex1 from an offline visual localization prototype into a real-time GNSS-denied navigation algorithm. The extended system is based on two complementary sources of geographic information:

1. Predefined annotated previous drone videos.
2. GIS datasets such as satellite imagery, orthophotos, elevation data, road maps, and building layers.

The previous-video database provides viewpoint-specific visual information from the drone’s actual flight environment. The GIS database provides a wider geographic reference that is not limited to previously flown paths. Combining these two sources allows the drone to localize itself in real time even when GNSS is unavailable.

#### System Overview

The proposed real-time system contains two main stages:

1. Offline preprocessing stage.
2. Online real-time navigation stage.

The offline stage builds a unified visual-geographic reference database. The online stage receives live drone video and telemetry, matches the current frame against the reference database and GIS map, estimates the camera-center ground coordinate, and updates the navigation state over time.

The high-level architecture is:

```text
Offline preprocessing
---------------------
Previous drone videos + SRT telemetry
        ↓
Geo-referenced video frame database

GIS datasets
satellite imagery / orthophoto / DEM / roads / buildings
        ↓
Geo-referenced map tile database

Video database + GIS database
        ↓
Unified visual-geographic reference database


Online real-time navigation
---------------------------
Live drone frame + altitude + camera angle + IMU/yaw if available
        ↓
Feature extraction and optional semantic segmentation
        ↓
Candidate retrieval from previous-video database and GIS map tiles
        ↓
Feature matching and geometric verification
        ↓
Camera-center coordinate estimation
        ↓
Temporal filtering and confidence estimation
        ↓
Real-time position estimate
```

#### Offline Preprocessing Stage

The offline preprocessing stage prepares the data before the drone performs GNSS-denied navigation.

##### 1. Previous Video Preprocessing

The system receives previously recorded drone videos with SRT telemetry. These videos are treated as annotated reference flights because the SRT files contain GNSS coordinates and altitude.

For each video:

1. Extract sampled frames.
2. Parse the SRT telemetry.
3. Extract GNSS latitude, longitude, relative altitude, and timestamps.
4. Estimate heading from GNSS movement if yaw is unavailable.
5. Use the camera angle and altitude to compute the camera-center ground coordinate.
6. Extract visual descriptors from every frame.
7. Optionally extract semantic labels such as roads, buildings, trees, roundabouts, and pedestrian crossings.
8. Store the result in the reference database.

Each stored reference item contains:

```text
frame image
timestamp
source video name
drone latitude and longitude
altitude
estimated heading
camera depression angle
camera-center latitude and longitude
visual descriptors
semantic labels, if available
confidence metadata
```

This part is already implemented in the Ex1 prototype using videos v11, v12, and v13.

##### 2. GIS Dataset Preprocessing

In addition to previous drone videos, the system uses GIS data. The GIS data may include:

* Satellite imagery or orthophoto tiles.
* Digital elevation model, DEM.
* Road network.
* Building footprints.
* Land-use or vegetation layers.
* Manually annotated landmarks.

The map area is divided into small geo-referenced tiles. Each tile has known geographic bounds. For every tile, the system stores:

```text
tile image
tile geographic bounds
tile center latitude and longitude
scale / ground sampling distance
visual descriptors
semantic/vector annotations
elevation data, if available
```

The GIS database complements the previous-video database. If the live drone image does not match a previous flight frame, it may still match a satellite or orthophoto tile.

##### 3. Unified Reference Database

The final offline product is a unified database containing two types of reference items:

1. Drone-frame references from previous videos.
2. GIS/map-tile references from satellite or orthophoto data.

Each database entry is indexed by both visual descriptors and geographic metadata. This allows the online system to quickly retrieve candidate locations.

A possible database schema is:

```text
reference_id
reference_type: video_frame / gis_tile
image_path
source_name
timestamp, if video frame
latitude
longitude
geo_bounds, if map tile
altitude, if video frame
heading, if available
camera_angle, if available
semantic_labels
descriptor_path
quality_score
```

#### Online Real-Time Navigation Stage

The online stage runs during flight. The drone receives a live video stream and telemetry, but no GNSS coordinates.

For every incoming frame:

1. Read the current video frame.
2. Read available telemetry such as altitude, camera angle, IMU yaw, and barometer height.
3. Extract visual features from the frame.
4. Optionally perform semantic segmentation.
5. Retrieve candidate locations from the previous-video database.
6. Retrieve candidate locations from the GIS map-tile database.
7. Match the query frame to candidate references.
8. Use geometric verification to reject weak matches.
9. Estimate the camera-center ground coordinate.
10. Fuse the estimate with previous estimates using a temporal filter.
11. Output the estimated location and confidence score.

#### Candidate Retrieval

A real-time system cannot compare every live frame against every stored reference image. Therefore, candidate retrieval is used.

The system first performs a fast coarse search. Possible retrieval methods include:

* Global image descriptors.
* Bag-of-visual-words.
* CNN or Vision Transformer embeddings.
* GPS-prior window if approximate initial position is available.
* Motion-prior window from the previous estimated position.

The output of this step is a small set of candidate reference images or map tiles.

For example:

```text
Live frame
        ↓
global descriptor search
        ↓
top 20 candidate video frames
top 20 candidate GIS tiles
```

Only these candidates are used in the more expensive feature matching stage.

#### Feature Matching and Geometric Verification

After candidate retrieval, the system performs local feature matching. The baseline implementation uses ORB features because they are fast and available in OpenCV. A stronger real-time version can replace ORB with learned matchers such as SuperPoint with LightGlue.

For each candidate:

1. Match descriptors between the query frame and candidate image.
2. Apply Lowe’s ratio test or mutual matching.
3. Estimate homography using RANSAC.
4. Count the number of homography inliers.
5. Compute a confidence score.

The selected match is the candidate with the strongest geometrically verified score.

A simple confidence score is:

```text
score = 10 · homography_inliers + number_of_good_matches
```

The system rejects matches when the confidence score is too low.

#### Coordinate Estimation

There are two possible cases.

##### Case 1: Match to Previous Drone Video Frame

If the query frame matches a previous drone frame, the baseline estimate is the camera-center coordinate of the matched reference frame.

A better version uses the homography between the query frame and the reference frame. The homography can estimate the displacement between the query image center and the matched reference image center. This pixel displacement can be converted to meters using altitude, camera angle, and camera field of view.

The improved estimate is:

```text
reference camera-center coordinate
        +
homography-based local displacement
        =
query camera-center coordinate
```

##### Case 2: Match to GIS / Satellite Tile

If the query frame matches a geo-referenced GIS tile, the coordinate can be estimated directly from the tile geometry.

Since the GIS tile has known geographic bounds, the matched query center can be mapped into the tile coordinate system using the estimated homography. The tile pixel coordinate is then converted to latitude and longitude.

The estimate is:

```text
query image center
        ↓
homography to GIS tile
        ↓
pixel coordinate inside geo-referenced tile
        ↓
latitude and longitude
```

This is one of the main advantages of using GIS data: the map tile already has global coordinates.

#### Temporal Filtering

Frame-by-frame visual localization can be noisy. Therefore, the real-time system should not use each frame independently. Instead, it should maintain a navigation state over time.

The state may include:

```text
latitude
longitude
velocity north/east
heading
confidence
```

A Kalman filter or particle filter can combine:

* Previous estimated position.
* Current visual localization estimate.
* IMU/yaw measurements.
* Barometric altitude.
* Expected drone motion limits.
* Match confidence.

If the current match has high confidence, it strongly updates the state. If the match has low confidence, it is rejected or given low weight.

#### Confidence and Failure Handling

The system must detect when visual localization is unreliable. Confidence can be based on:

* Number of homography inliers.
* Ratio between the best and second-best candidate scores.
* Spatial consistency with the previous position.
* Agreement between previous-video match and GIS-tile match.
* Semantic consistency, for example roads matching roads and buildings matching buildings.
* Temporal smoothness.

If confidence is low, the system should not output a false precise location. Instead, it should output:

```text
position estimate: unavailable or low confidence
last reliable position
estimated drift region
```

This is important for safety because an incorrect high-confidence estimate is more dangerous than no estimate.

#### Semantic Annotations

Annotated previous videos and GIS layers can improve matching.

For example, the system can use semantic classes such as:

* road
* building
* tree
* roundabout
* parking lot
* pedestrian crossing
* antenna
* field

Semantic information can be used in three ways:

1. Candidate filtering
   If the query frame contains a road intersection, prefer map tiles or previous frames that also contain a road intersection.

2. Match validation
   Reject matches where semantic regions do not align, for example road matched to trees.

3. Landmark-based localization
   Stable landmarks such as road intersections, roundabouts, and large buildings can be used as high-confidence anchors.

This is especially useful in areas with repetitive low-level textures, where local feature matching alone may produce false matches.

#### Real-Time Constraints

A real-time navigation algorithm must control runtime. The following design choices support real-time performance:

1. Process one keyframe every fixed interval instead of every video frame.
2. Use global descriptors for fast candidate retrieval.
3. Use local feature matching only on a small top-k candidate set.
4. Cache descriptors for all reference frames and GIS tiles.
5. Use confidence filtering to avoid unnecessary refinement on weak candidates.
6. Use temporal filtering to interpolate between visual updates.
7. Run expensive models, such as segmentation or learned matching, only on keyframes.

The real-time loop can therefore run as:

```text
For each new frame:
    if frame is not a keyframe:
        propagate position using motion model
    else:
        retrieve candidates
        match candidates
        verify geometry
        estimate position
        update temporal filter
```

#### Relation to the Implemented Ex1 Prototype

The implemented Ex1 system already provides the previous-video localization branch of the final algorithm.

Implemented components:

```text
SRT parsing
camera-center projection
reference database from v11-v13
v14 GNSS-denied query simulation
ORB visual matching
homography verification
confidence filtering
evaluation against SRT ground truth
```

The final project extension adds the GIS branch:

```text
GIS/satellite/orthophoto map tiles
geo-referenced tile database
map-tile matching
coordinate estimation from tile geometry
semantic/vector GIS validation
```

Therefore, the final project is a natural extension of Ex1. The current implementation demonstrates the core mechanism using previous annotated videos, and the proposed GIS extension turns it into a more general real-time navigation design.

#### Proposed Real-Time Algorithm

```text
Algorithm: Real-Time Visual Navigation with Previous Videos and GIS

Offline:
1. For each annotated drone video:
       extract frames
       parse SRT telemetry
       compute camera-center coordinates
       extract visual descriptors
       store in video reference database

2. For each GIS map tile:
       load geo-referenced satellite/orthophoto tile
       extract visual descriptors
       load vector annotations if available
       store in GIS reference database

3. Build search index over all descriptors.

Online:
1. Initialize navigation state.

2. For each incoming video frame:
       read altitude, camera angle, IMU/yaw if available
       extract global descriptor
       retrieve top-k video candidates and top-k GIS candidates
       extract local features
       match local features to candidates
       estimate homography with RANSAC
       reject weak matches
       estimate camera-center coordinate
       fuse estimate with temporal filter
       output coordinate and confidence

3. If no reliable match is found:
       propagate state using motion model
       lower confidence
       wait for next keyframe
```

#### Expected Advantages

The combined previous-video and GIS approach has several advantages:

1. Previous drone videos provide viewpoint-specific information close to the real camera perspective.
2. GIS data provides geographic coverage beyond previously flown trajectories.
3. Semantic GIS layers help reject visually similar but geographically wrong matches.
4. Confidence filtering improves reliability.
5. Temporal filtering supports continuous navigation between visual updates.

#### Expected Limitations

The final system also has limitations:

1. Satellite imagery may differ from drone imagery due to viewpoint, resolution, shadows, season, and time of capture.
2. GIS data may be outdated.
3. Repetitive urban or agricultural patterns may cause false matches.
4. Real-time learned matching may require GPU acceleration.
5. Flat-ground projection may fail in terrain with strong elevation changes.
6. A high-quality DEM is needed for accurate ray-ground intersection.
7. The system must handle low-confidence frames safely.

Despite these limitations, the proposed design provides a realistic architecture for GNSS-denied visual navigation based on previous annotated drone videos and GIS datasets.

---

### 13. Future Work

Several improvements can be added in future versions:

1. Use homography to refine the coordinate estimate
   Instead of copying the matched frame coordinate, use the homography to estimate how far the query frame center is shifted relative to the reference frame center.

2. Replace ORB with LightGlue or SuperGlue
   Learned feature matchers are expected to improve matching robustness under viewpoint and illumination changes.

3. Add temporal smoothing
   A Kalman filter or particle filter can reduce jumps between consecutive frames.

4. Add semantic segmentation
   Segmenting roads, buildings, trees, and other classes can help reject bad matches and focus on stable landmarks.

5. Use satellite or orthomosaic maps
   Matching query frames against a geo-referenced orthomosaic or satellite map would allow localization beyond the previously recorded flight paths.

6. Use a digital elevation model
   A DEM would replace the flat-ground projection and improve camera-center geolocation.

7. Evaluate more videos
   Additional videos from different heights, camera angles, and lighting conditions would provide a more complete analysis.

---

### 14. Conclusion

This project implemented a complete prototype for GNSS-denied visual navigation using drone video and SRT telemetry.

First, a geometry-based preliminary experiment was implemented. It parsed SRT telemetry, used relative altitude and a known 60-degree camera angle, estimated heading from GNSS movement, and computed the expected camera-center ground path. The projected path was compared with the captured GNSS path.

Second, a preprocessing stage was implemented. It created a geo-referenced reference database from videos `v11`, `v12`, and `v13`. The database contained 541 reference frames with image paths, timestamps, altitude, heading, and estimated camera-center coordinates.

Third, a GNSS-denied visual localization experiment was performed. Video `v14` was treated as the query video without GNSS. ORB features and homography scoring were used to match query frames against the reference database. The baseline achieved a mean error of 183.25 meters and a median error of 98.48 meters.

Finally, a confidence-filtering improvement was added. By accepting only matches with at least 10 homography inliers, the mean error was reduced to 50.27 meters and the median error to 48.28 meters, while keeping 42.9% of the query frames.

The results show that visual matching can support GNSS-denied localization, but robust navigation requires confidence estimation, temporal smoothing, and stronger matching methods. The implemented system satisfies the assignment requirements and provides a clear baseline for future improvements.

---

### References

[1] Campos, C., Elvira, R., Gómez Rodríguez, J. J., Montiel, J. M. M., and Tardós, J. D. ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM.

[2] Kern, A., Bobbe, M., Khedar, Y., and Bestmann, U. OpenREALM: Real-time Mapping for Unmanned Aerial Vehicles.

[3] Xu, W., Yao, Y., Cao, J., Wei, Z., Liu, C., Wang, J., and Peng, M. UAV-VisLoc: A Large-scale Dataset for UAV Visual Localization.

[4] Gurgu, M. M., Queralta, J. P., and Westerlund, T. Vision-based GNSS-Free Localization for UAVs in the Wild.

[5] OpenAthena project. Drone image geolocation using sensor metadata and digital elevation models.

[6] Sarlin, P. E., DeTone, D., Malisiewicz, T., and Rabinovich, A. SuperGlue: Learning Feature Matching with Graph Neural Networks.

[7] Lindenberger, P., Sarlin, P. E., and Pollefeys, M. LightGlue: Local Feature Matching at Light Speed.

[8] Spasev, V., Dimitrovski, I., Chorbev, I., and Kitanovski, I. Semantic Segmentation of Unmanned Aerial Vehicle Remote Sensing Images using SegFormer.
