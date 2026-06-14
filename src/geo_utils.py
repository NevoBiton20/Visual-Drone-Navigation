"""Geographic helper functions for the visual drone navigation assignment.

The implementation intentionally avoids heavy GIS dependencies so the baseline
can run in a normal university Python environment.
"""

from __future__ import annotations

import math
from typing import Tuple

EARTH_RADIUS_M = 6371008.8


def deg2rad(deg: float) -> float:
    return deg * math.pi / 180.0


def rad2deg(rad: float) -> float:
    return rad * 180.0 / math.pi


def destination_point(lat_deg: float, lon_deg: float, bearing_deg: float, distance_m: float) -> Tuple[float, float]:
    """Move from (lat, lon) by distance and bearing on a spherical Earth.

    Args:
        lat_deg: start latitude in degrees.
        lon_deg: start longitude in degrees.
        bearing_deg: bearing clockwise from north in degrees.
        distance_m: distance in meters.

    Returns:
        (latitude, longitude) in degrees.
    """
    if distance_m == 0 or math.isnan(distance_m):
        return lat_deg, lon_deg

    lat1 = deg2rad(lat_deg)
    lon1 = deg2rad(lon_deg)
    brng = deg2rad(bearing_deg)
    angular_distance = distance_m / EARTH_RADIUS_M

    lat2 = math.asin(
        math.sin(lat1) * math.cos(angular_distance)
        + math.cos(lat1) * math.sin(angular_distance) * math.cos(brng)
    )
    lon2 = lon1 + math.atan2(
        math.sin(brng) * math.sin(angular_distance) * math.cos(lat1),
        math.cos(angular_distance) - math.sin(lat1) * math.sin(lat2),
    )

    lon2 = (lon2 + 3 * math.pi) % (2 * math.pi) - math.pi
    return rad2deg(lat2), rad2deg(lon2)


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance in meters."""
    phi1, phi2 = deg2rad(lat1), deg2rad(lat2)
    dphi = deg2rad(lat2 - lat1)
    dlambda = deg2rad(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Initial bearing from point 1 to point 2, clockwise from north."""
    phi1, phi2 = deg2rad(lat1), deg2rad(lat2)
    dlambda = deg2rad(lon2 - lon1)
    x = math.sin(dlambda) * math.cos(phi2)
    y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlambda)
    return (rad2deg(math.atan2(x, y)) + 360) % 360


def meters_per_pixel_ground(altitude_m: float, fov_deg: float, image_size_px: int) -> float:
    """Approximate nadir ground sampling distance for a given FOV.

    This is a first-order estimate and is most accurate for near-nadir views and flat ground.
    """
    footprint_m = 2.0 * altitude_m * math.tan(deg2rad(fov_deg) / 2.0)
    return footprint_m / image_size_px
