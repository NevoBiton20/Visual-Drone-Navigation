"""Plotting utilities for path visualization."""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


def plot_paths(df: pd.DataFrame, out_png: str | Path, title: str = "Drone path vs camera-center ground path") -> None:
    """Plot GNSS drone path and projected camera-center path in lon/lat coordinates."""
    out_png = Path(out_png)
    out_png.parent.mkdir(parents=True, exist_ok=True)

    fig = plt.figure(figsize=(9, 7))
    if {"longitude", "latitude"}.issubset(df.columns):
        plt.plot(df["longitude"], df["latitude"], marker=".", linewidth=1, label="Drone GNSS path")
    if {"center_longitude", "center_latitude"}.issubset(df.columns):
        plt.plot(df["center_longitude"], df["center_latitude"], marker=".", linewidth=1, label="Projected camera-center path")
    plt.xlabel("Longitude")
    plt.ylabel("Latitude")
    plt.title(title)
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    fig.savefig(out_png, dpi=180)
    plt.close(fig)


def save_kml(df: pd.DataFrame, out_kml: str | Path) -> None:
    """Save a simple KML with drone path and center-look path for Google Earth."""
    out_kml = Path(out_kml)
    out_kml.parent.mkdir(parents=True, exist_ok=True)

    def coord_lines(lon_col: str, lat_col: str) -> str:
        coords = []
        for _, row in df.iterrows():
            if pd.notna(row.get(lon_col)) and pd.notna(row.get(lat_col)):
                coords.append(f"{row[lon_col]},{row[lat_col]},0")
        return "\n".join(coords)

    drone_coords = coord_lines("longitude", "latitude")
    center_coords = coord_lines("center_longitude", "center_latitude")
    kml = f'''<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
<Document>
  <name>Visual Drone Navigation Paths</name>
  <Placemark>
    <name>Drone GNSS path</name>
    <LineString><coordinates>{drone_coords}</coordinates></LineString>
  </Placemark>
  <Placemark>
    <name>Projected camera-center path</name>
    <LineString><coordinates>{center_coords}</coordinates></LineString>
  </Placemark>
</Document>
</kml>
'''
    out_kml.write_text(kml, encoding="utf-8")
