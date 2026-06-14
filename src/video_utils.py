"""Video frame extraction utilities."""

from __future__ import annotations

from pathlib import Path
from typing import List

import cv2
import pandas as pd
from tqdm import tqdm


def extract_frames(video_path: str | Path, out_dir: str | Path, frame_step: int = 30, max_frames: int | None = None) -> pd.DataFrame:
    """Extract frames from a video file.

    Args:
        video_path: input video file.
        out_dir: folder to write .jpg frames.
        frame_step: save one frame every N frames.
        max_frames: optional limit on saved frames.

    Returns:
        DataFrame with frame_index, timestamp_s, image_path.
    """
    video_path = Path(video_path)
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise FileNotFoundError(f"Could not open video: {video_path}")

    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    rows: List[dict] = []
    saved = 0

    for frame_idx in tqdm(range(total), desc=f"Extracting {video_path.name}"):
        ok, frame = cap.read()
        if not ok:
            break
        if frame_idx % frame_step != 0:
            continue
        image_path = out_dir / f"{video_path.stem}_frame_{frame_idx:06d}.jpg"
        cv2.imwrite(str(image_path), frame)
        rows.append({
            "video": video_path.name,
            "frame_index": frame_idx,
            "timestamp_s": frame_idx / fps,
            "image_path": str(image_path),
        })
        saved += 1
        if max_frames is not None and saved >= max_frames:
            break

    cap.release()
    return pd.DataFrame(rows)
