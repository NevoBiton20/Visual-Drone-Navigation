"""Camera-to-ground projection utilities.

Baseline assumption: flat ground. This is enough for the assignment's first
experiment and can later be replaced by a DEM/raycast implementation.
"""

from __future__ import annotations

import math
from typing import Optional

import numpy as np
import pandas as pd

from .geo_utils import bearing_deg, destination_point


def normalize_heading(deg: float) -> float:
    return deg % 360.0


def depression_angle_from_pitch(pitch_deg: Optional[float], fallback_pitch_deg: float = -60.0, convention: str = "dji") -> float:
    """Convert camera/gimbal pitch to depression angle below horizon.

    DJI convention is usually:
        0 deg  = camera looks at horizon
        -90 deg = camera looks straight down

    Therefore pitch=-60 means a 60-degree depression angle.
    """
    if pitch_deg is None or (isinstance(pitch_deg, float) and math.isnan(pitch_deg)):
        pitch_deg = fallback_pitch_deg

    if convention.lower() == "dji":
        depression = abs(float(pitch_deg))
    else:
        depression = float(pitch_deg)

    # Keep away from degenerate horizon/negative values.
    return max(1e-3, min(89.999, depression))


def center_ground_offset_m(altitude_m: float, depression_deg: float) -> float:
    """Horizontal ground offset from drone to center of camera view.

    For a flat-ground model:
        offset = altitude / tan(depression)

    depression=90 means nadir view, offset≈0.
    depression=45 means center point is roughly altitude meters ahead.
    """
    return altitude_m / math.tan(math.radians(depression_deg))


def fill_missing_heading(df: pd.DataFrame) -> pd.Series:
    """Return a heading series, using yaw/heading if available or GNSS bearing otherwise."""
    if "heading_deg" in df and df["heading_deg"].notna().any():
        heading = df["heading_deg"].copy()
    elif "yaw_deg" in df and df["yaw_deg"].notna().any():
        heading = df["yaw_deg"].copy()
    elif "gimbal_yaw_deg" in df and df["gimbal_yaw_deg"].notna().any():
        heading = df["gimbal_yaw_deg"].copy()
    else:
        heading = pd.Series([np.nan] * len(df), index=df.index, dtype=float)

    # Infer missing values from consecutive GPS points.
    if {"latitude", "longitude"}.issubset(df.columns):
        for i in range(len(df) - 1):
            if pd.isna(heading.iloc[i]) and pd.notna(df.loc[i, "latitude"]) and pd.notna(df.loc[i + 1, "latitude"]):
                heading.iloc[i] = bearing_deg(
                    df.loc[i, "latitude"],
                    df.loc[i, "longitude"],
                    df.loc[i + 1, "latitude"],
                    df.loc[i + 1, "longitude"],
                )
        if len(df) > 1 and pd.isna(heading.iloc[-1]):
            heading.iloc[-1] = heading.iloc[-2]

    heading = heading.ffill().bfill().fillna(0.0)
    return heading.apply(normalize_heading)


def choose_altitude_m(row: pd.Series, fallback_altitude_m: float = 119.0) -> float:
    """Choose the best available height above local ground."""
    for col in ["rel_alt_m", "barometer_m", "abs_alt_m"]:
        if col in row and pd.notna(row[col]):
            return max(0.1, float(row[col]))
    return float(fallback_altitude_m)


def add_center_ground_projection(
    df: pd.DataFrame,
    fallback_pitch_deg: float = -60.0,
    fallback_altitude_m: float = 119.0,
    pitch_convention: str = "dji",
) -> pd.DataFrame:
    """Add projected camera-center ground point to telemetry DataFrame."""
    if df.empty:
        return df.copy()

    out = df.copy()
    out["used_heading_deg"] = fill_missing_heading(out)

    center_lats = []
    center_lons = []
    altitudes = []
    depressions = []
    offsets = []

    for _, row in out.iterrows():
        lat, lon = row.get("latitude"), row.get("longitude")
        if pd.isna(lat) or pd.isna(lon):
            center_lats.append(np.nan)
            center_lons.append(np.nan)
            altitudes.append(np.nan)
            depressions.append(np.nan)
            offsets.append(np.nan)
            continue

        altitude_m = choose_altitude_m(row, fallback_altitude_m)
        pitch = row.get("gimbal_pitch_deg") if "gimbal_pitch_deg" in out.columns else None
        depression = depression_angle_from_pitch(pitch, fallback_pitch_deg, pitch_convention)
        offset = center_ground_offset_m(altitude_m, depression)
        clat, clon = destination_point(float(lat), float(lon), float(row["used_heading_deg"]), offset)

        center_lats.append(clat)
        center_lons.append(clon)
        altitudes.append(altitude_m)
        depressions.append(depression)
        offsets.append(offset)

    out["used_altitude_m"] = altitudes
    out["camera_depression_deg"] = depressions
    out["center_offset_m"] = offsets
    out["center_latitude"] = center_lats
    out["center_longitude"] = center_lons
    return out
