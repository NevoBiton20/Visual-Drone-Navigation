# Algorithm Design Summary

## Preprocessing Stage

Input: videos + SRT telemetry with GNSS.

Output: reference database containing frame paths and geo-referenced camera-center coordinates.

Pseudocode:

```text
for each training_video, training_srt:
    frames = extract_frames(training_video, every N frames)
    telemetry = parse_srt(training_srt)
    projected = []

    for each telemetry record:
        lat, lon = GNSS drone position
        h = relative altitude or barometric height
        heading = yaw/heading or bearing from consecutive GNSS points
        pitch = gimbal pitch
        depression = abs(pitch)        # DJI convention
        offset = h / tan(depression)
        center_lat, center_lon = move(lat, lon, heading, offset)
        projected.append(center_lat, center_lon)

    synchronize frames with telemetry by timestamp
    save frame metadata + coordinates to reference database
```

## Navigation Stage

Input: real-time video + telemetry without GNSS.

Output: estimated coordinate of center ground point.

Pseudocode:

```text
for each query frame:
    features_query = extract_features(query frame)
    candidates = retrieve visually similar reference frames

    best_score = -inf
    for candidate in candidates:
        matches = match_features(query, candidate)
        H, inliers = estimate_homography(matches)
        if inliers > best_score:
            best_candidate = candidate
            best_score = inliers
            best_H = H

    estimated_center = best_candidate.center_coordinate
    estimated_center = optional_homography_refinement(estimated_center, best_H)
    smoothed_center = temporal_filter(estimated_center)
    output smoothed_center
```
