"""Robust-enough DJI/Autel SRT telemetry parser.

Drone SRT files vary by model and firmware. This parser searches each subtitle
block for common telemetry keys, including GPS coordinates, relative/absolute
altitude, barometer, yaw/heading, and gimbal pitch.

It is designed for the university exercise baseline, not as a complete vendor
SDK replacement.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Dict, List, Optional

import pandas as pd

_TIME_LINE_RE = re.compile(r"(\d{2}:\d{2}:\d{2},\d{3})\s+-->\s+(\d{2}:\d{2}:\d{2},\d{3})")
_FLOAT_RE = r"[-+]?\d+(?:\.\d+)?"


@dataclass
class TelemetryRecord:
    block_index: int
    start_time: str
    end_time: str
    timestamp_s: float
    frame_index: Optional[int] = None
    latitude: Optional[float] = None
    longitude: Optional[float] = None
    rel_alt_m: Optional[float] = None
    abs_alt_m: Optional[float] = None
    barometer_m: Optional[float] = None
    yaw_deg: Optional[float] = None
    heading_deg: Optional[float] = None
    gimbal_pitch_deg: Optional[float] = None
    gimbal_yaw_deg: Optional[float] = None
    gimbal_roll_deg: Optional[float] = None
    raw_text: str = ""


def _time_to_seconds(t: str) -> float:
    hh, mm, rest = t.split(":")
    ss, ms = rest.split(",")
    return int(hh) * 3600 + int(mm) * 60 + int(ss) + int(ms) / 1000.0


def _find_float(patterns: List[str], text: str) -> Optional[float]:
    for pat in patterns:
        m = re.search(pat, text, flags=re.IGNORECASE)
        if m:
            try:
                return float(m.group(1))
            except Exception:
                continue
    return None


def _extract_lat_lon(text: str) -> tuple[Optional[float], Optional[float]]:
    # Common DJI formats include [latitude: 32.123] [longitude: 35.123]
    lat = _find_float([
        rf"latitude\s*[:=]\s*({_FLOAT_RE})",
        rf"\[lat\s*[:=]\s*({_FLOAT_RE})\]",
        rf"GPS\s*\(\s*({_FLOAT_RE})\s*,",
    ], text)
    lon = _find_float([
        rf"longitude\s*[:=]\s*({_FLOAT_RE})",
        rf"\[lon\s*[:=]\s*({_FLOAT_RE})\]",
        rf"GPS\s*\(\s*{_FLOAT_RE}\s*,\s*({_FLOAT_RE})",
    ], text)

    # Some SRTs have a compact format like [GPS: 32.1, 35.2, 100.0]
    if lat is None or lon is None:
        m = re.search(rf"GPS[^\d\-+]*({_FLOAT_RE})\s*[, ]\s*({_FLOAT_RE})", text, flags=re.IGNORECASE)
        if m:
            lat = float(m.group(1))
            lon = float(m.group(2))

    return lat, lon


def parse_srt(srt_path: str | Path) -> pd.DataFrame:
    """Parse an SRT file into a pandas DataFrame."""
    srt_path = Path(srt_path)
    content = srt_path.read_text(encoding="utf-8", errors="ignore")
    blocks = re.split(r"\n\s*\n", content.strip())
    records: List[TelemetryRecord] = []

    for block in blocks:
        lines = [ln.strip() for ln in block.splitlines() if ln.strip()]
        if not lines:
            continue

        block_index = -1
        time_line = None
        raw_text = " ".join(lines)

        if lines[0].isdigit():
            block_index = int(lines[0])
            for ln in lines[1:3]:
                if _TIME_LINE_RE.search(ln):
                    time_line = ln
                    break
        else:
            for ln in lines[:3]:
                if _TIME_LINE_RE.search(ln):
                    time_line = ln
                    break

        if time_line is None:
            continue

        mtime = _TIME_LINE_RE.search(time_line)
        assert mtime is not None
        start_time, end_time = mtime.group(1), mtime.group(2)
        timestamp_s = _time_to_seconds(start_time)
        lat, lon = _extract_lat_lon(raw_text)

        frame_index = _find_float([
            rf"frame(?:_cnt|_index|cnt)?\s*[:=]\s*(\d+)",
            rf"FrameCnt\s*[:=]\s*(\d+)",
            rf"fnum\s*[:=]\s*(\d+)",
        ], raw_text)

        rec = TelemetryRecord(
            block_index=block_index if block_index >= 0 else len(records) + 1,
            start_time=start_time,
            end_time=end_time,
            timestamp_s=timestamp_s,
            frame_index=int(frame_index) if frame_index is not None else None,
            latitude=lat,
            longitude=lon,
            rel_alt_m=_find_float([
                rf"rel[_ ]?alt(?:itude)?\s*[:=]\s*({_FLOAT_RE})",
                rf"relative[_ ]?alt(?:itude)?\s*[:=]\s*({_FLOAT_RE})",
                rf"height\s*[:=]\s*({_FLOAT_RE})\s*m",
                rf"H\s*[:=]\s*({_FLOAT_RE})\s*m",
            ], raw_text),
            abs_alt_m=_find_float([
                rf"abs[_ ]?alt(?:itude)?\s*[:=]\s*({_FLOAT_RE})",
                rf"alt(?:itude)?\s*[:=]\s*({_FLOAT_RE})\s*m",
            ], raw_text),
            barometer_m=_find_float([
                rf"barometer\s*[:=]\s*({_FLOAT_RE})",
                rf"baro\s*[:=]\s*({_FLOAT_RE})",
            ], raw_text),
            yaw_deg=_find_float([
                rf"yaw\s*[:=]\s*({_FLOAT_RE})",
                rf"flight_yaw\s*[:=]\s*({_FLOAT_RE})",
            ], raw_text),
            heading_deg=_find_float([
                rf"heading\s*[:=]\s*({_FLOAT_RE})",
                rf"compass\s*[:=]\s*({_FLOAT_RE})",
            ], raw_text),
            gimbal_pitch_deg=_find_float([
                rf"gimbal[_ ]?pitch\s*[:=]\s*({_FLOAT_RE})",
                rf"gb_pitch\s*[:=]\s*({_FLOAT_RE})",
                rf"pitch\s*[:=]\s*({_FLOAT_RE})",
            ], raw_text),
            gimbal_yaw_deg=_find_float([
                rf"gimbal[_ ]?yaw\s*[:=]\s*({_FLOAT_RE})",
                rf"gb_yaw\s*[:=]\s*({_FLOAT_RE})",
            ], raw_text),
            gimbal_roll_deg=_find_float([
                rf"gimbal[_ ]?roll\s*[:=]\s*({_FLOAT_RE})",
                rf"gb_roll\s*[:=]\s*({_FLOAT_RE})",
                rf"roll\s*[:=]\s*({_FLOAT_RE})",
            ], raw_text),
            raw_text=raw_text,
        )
        records.append(rec)

    df = pd.DataFrame([asdict(r) for r in records])
    if not df.empty:
        df = df.sort_values("timestamp_s").reset_index(drop=True)
    return df


def parse_srt_to_csv(srt_path: str | Path, out_csv: str | Path) -> pd.DataFrame:
    df = parse_srt(srt_path)
    Path(out_csv).parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(out_csv, index=False)
    return df


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Parse DJI/Autel SRT telemetry into CSV")
    parser.add_argument("srt", help="Path to SRT file")
    parser.add_argument("--out", default="outputs/telemetry.csv", help="Output CSV path")
    args = parser.parse_args()

    parsed = parse_srt_to_csv(args.srt, args.out)
    print(f"Parsed {len(parsed)} telemetry records -> {args.out}")
