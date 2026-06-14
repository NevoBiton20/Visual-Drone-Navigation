"""Classical visual matching utilities.

This file intentionally uses ORB as the default because it is simple, fast, and
available in standard OpenCV. The report recommends replacing/augmenting it with
SuperPoint+LightGlue for the stronger version.
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, Tuple

import cv2
import numpy as np


def load_gray(path: str | Path, max_width: int = 960) -> np.ndarray:
    img = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(path)
    h, w = img.shape[:2]
    if w > max_width:
        scale = max_width / w
        img = cv2.resize(img, (int(w * scale), int(h * scale)), interpolation=cv2.INTER_AREA)
    return img


def orb_features(path: str | Path, nfeatures: int = 3000) -> Tuple[np.ndarray, np.ndarray]:
    img = load_gray(path)
    orb = cv2.ORB_create(nfeatures=nfeatures)
    keypoints, descriptors = orb.detectAndCompute(img, None)
    if descriptors is None or len(keypoints) == 0:
        return np.zeros((0, 2), dtype=np.float32), np.zeros((0, 32), dtype=np.uint8)
    pts = np.array([kp.pt for kp in keypoints], dtype=np.float32)
    return pts, descriptors


def match_orb(desc1: np.ndarray, desc2: np.ndarray, ratio: float = 0.75) -> list[cv2.DMatch]:
    if desc1 is None or desc2 is None or len(desc1) < 2 or len(desc2) < 2:
        return []
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
    knn = matcher.knnMatch(desc1, desc2, k=2)
    good = []
    for pair in knn:
        if len(pair) < 2:
            continue
        m, n = pair
        if m.distance < ratio * n.distance:
            good.append(m)
    return good


def estimate_homography(pts1: np.ndarray, pts2: np.ndarray, matches: list[cv2.DMatch], ransac_thresh: float = 5.0) -> Dict:
    """Estimate homography from image1 to image2.

    Returns a dict with H, inlier count, and rough pixel translation of image center.
    """
    if len(matches) < 4:
        return {"H": None, "num_matches": len(matches), "num_inliers": 0, "dx_px": None, "dy_px": None}

    src = np.float32([pts1[m.queryIdx] for m in matches]).reshape(-1, 1, 2)
    dst = np.float32([pts2[m.trainIdx] for m in matches]).reshape(-1, 1, 2)
    H, mask = cv2.findHomography(src, dst, cv2.RANSAC, ransac_thresh)
    inliers = int(mask.sum()) if mask is not None else 0

    dx = dy = None
    if H is not None:
        # Translate center of a normalized 960-width image approximately.
        cx, cy = 480.0, 270.0
        p = np.array([[[cx, cy]]], dtype=np.float32)
        q = cv2.perspectiveTransform(p, H)[0, 0]
        dx, dy = float(q[0] - cx), float(q[1] - cy)

    return {"H": H, "num_matches": len(matches), "num_inliers": inliers, "dx_px": dx, "dy_px": dy}


def pair_match_score(img1_path: str | Path, img2_path: str | Path, nfeatures: int = 3000) -> Dict:
    pts1, desc1 = orb_features(img1_path, nfeatures=nfeatures)
    pts2, desc2 = orb_features(img2_path, nfeatures=nfeatures)
    matches = match_orb(desc1, desc2)
    result = estimate_homography(pts1, pts2, matches)
    result["score"] = result["num_inliers"]
    return result
