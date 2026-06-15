import math
import pandas as pd
import matplotlib.pyplot as plt


INPUT_CSV = "outputs/v11/projected_camera_center_path.csv"
OUTPUT_CSV = "outputs/v11/clean_projected_camera_center_path.csv"
OUTPUT_PNG = "outputs/v11/clean_path_comparison.png"

MIN_ALTITUDE_M = 80
FRAME_STEP = 30
CAMERA_DEPRESSION_DEG = 60


def bearing_deg(lat1, lon1, lat2, lon2):
    """
    Compute bearing from point 1 to point 2 in degrees.
    0 = north, 90 = east.
    """
    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)
    dlon = math.radians(lon2 - lon1)

    x = math.sin(dlon) * math.cos(lat2)
    y = (
        math.cos(lat1) * math.sin(lat2)
        - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    )

    bearing = math.degrees(math.atan2(x, y))
    return (bearing + 360) % 360


def project_point(lat, lon, distance_m, heading_deg):
    """
    Move from lat/lon by distance_m in heading_deg direction.
    """
    earth_radius = 6378137.0

    lat1 = math.radians(lat)
    lon1 = math.radians(lon)
    bearing = math.radians(heading_deg)

    lat2 = math.asin(
        math.sin(lat1) * math.cos(distance_m / earth_radius)
        + math.cos(lat1) * math.sin(distance_m / earth_radius) * math.cos(bearing)
    )

    lon2 = lon1 + math.atan2(
        math.sin(bearing) * math.sin(distance_m / earth_radius) * math.cos(lat1),
        math.cos(distance_m / earth_radius) - math.sin(lat1) * math.sin(lat2),
    )

    return math.degrees(lat2), math.degrees(lon2)


def main():
    df = pd.read_csv(INPUT_CSV)

    # Keep only flying part
    df = df[df["rel_alt_m"] >= MIN_ALTITUDE_M].copy()

    # Sample fewer frames
    df = df.iloc[::FRAME_STEP].copy().reset_index(drop=True)

    # Compute heading from movement between sampled points
    headings = []

    for i in range(len(df)):
        if i < len(df) - 1:
            lat1, lon1 = df.loc[i, "latitude"], df.loc[i, "longitude"]
            lat2, lon2 = df.loc[i + 1, "latitude"], df.loc[i + 1, "longitude"]
        else:
            lat1, lon1 = df.loc[i - 1, "latitude"], df.loc[i - 1, "longitude"]
            lat2, lon2 = df.loc[i, "latitude"], df.loc[i, "longitude"]

        headings.append(bearing_deg(lat1, lon1, lat2, lon2))

    df["clean_heading_deg"] = headings

    # Smooth heading a little using rolling average
    df["clean_heading_deg"] = df["clean_heading_deg"].rolling(
        window=5,
        center=True,
        min_periods=1
    ).mean()

    # Recalculate offset and projected center
    depression_rad = math.radians(CAMERA_DEPRESSION_DEG)

    df["clean_center_offset_m"] = df["rel_alt_m"] / math.tan(depression_rad)

    center_lats = []
    center_lons = []

    for _, row in df.iterrows():
        center_lat, center_lon = project_point(
            row["latitude"],
            row["longitude"],
            row["clean_center_offset_m"],
            row["clean_heading_deg"]
        )
        center_lats.append(center_lat)
        center_lons.append(center_lon)

    df["clean_center_latitude"] = center_lats
    df["clean_center_longitude"] = center_lons

    df.to_csv(OUTPUT_CSV, index=False)

    plt.figure(figsize=(12, 8))

    plt.plot(
        df["longitude"],
        df["latitude"],
        marker="o",
        markersize=2,
        linewidth=1,
        label="Drone GNSS path"
    )

    plt.plot(
        df["clean_center_longitude"],
        df["clean_center_latitude"],
        marker="o",
        markersize=2,
        linewidth=1,
        label="Clean projected camera-center path"
    )

    plt.title("Clean drone path vs camera-center ground path")
    plt.xlabel("Longitude")
    plt.ylabel("Latitude")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(OUTPUT_PNG, dpi=200)

    print("Saved:")
    print(OUTPUT_CSV)
    print(OUTPUT_PNG)

    print("\nClean statistics:")
    print(df[[
        "rel_alt_m",
        "clean_heading_deg",
        "clean_center_offset_m",
        "latitude",
        "longitude",
        "clean_center_latitude",
        "clean_center_longitude"
    ]].describe())


if __name__ == "__main__":
    main()