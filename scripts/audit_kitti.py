#!/usr/bin/env python3
"""Audit a KITTI Raw synchronized drive without loading the full images/point clouds."""
from __future__ import annotations

import argparse
import json
import re
import statistics
from datetime import datetime
from pathlib import Path

STAMP_RE = re.compile(r"^(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d)(?:\.(\d{1,9}))?$")
EPOCH = datetime(1970, 1, 1)
STREAMS = {
    "oxts": ".txt",
    "velodyne_points": ".bin",
    "image_02": ".png",
    "image_03": ".png",
}
CALIB_FILES = (
    "calib_cam_to_cam.txt",
    "calib_imu_to_velo.txt",
    "calib_velo_to_cam.txt",
)


def timestamp_ns(value: str) -> int:
    """Parse KITTI timestamps at integer nanosecond precision (no float rounding)."""
    match = STAMP_RE.fullmatch(value.strip())
    if match is None:
        raise ValueError("Invalid KITTI timestamp: " + repr(value))
    date_text, frac = match.groups()
    dt = datetime.strptime(date_text, "%Y-%m-%d %H:%M:%S")
    elapsed = dt - EPOCH
    return ((elapsed.days * 86400 + elapsed.seconds) * 1_000_000_000
            + int((frac or "").ljust(9, "0") or "0"))


def _read_times(path: Path, errors: list[str]) -> list[int]:
    if not path.is_file():
        errors.append("Missing timestamps: " + str(path))
        return []
    try:
        lines = [line for line in path.read_text(encoding="utf-8").splitlines()
                 if line.strip()]
        times = [timestamp_ns(line) for line in lines]
    except (OSError, ValueError) as exc:
        errors.append("Unreadable timestamps " + str(path) + ": " + str(exc))
        return []
    if any(b <= a for a, b in zip(times, times[1:])):
        errors.append("Non-increasing timestamps: " + str(path))
    return times


def _check_samples(files: list[Path], kind: str, errors: list[str]) -> None:
    if not files:
        return
    sampled = [files[0], files[len(files) // 2], files[-1]]
    for path in dict.fromkeys(sampled):
        try:
            if kind == "velodyne_points":
                size = path.stat().st_size
                if size == 0 or size % 16 != 0:
                    errors.append("LiDAR .bin not float32 Nx4: " + str(path))
            elif kind.startswith("image_"):
                if path.open("rb").read(8) != b"\x89PNG\r\n\x1a\n":
                    errors.append("Invalid PNG signature: " + str(path))
            elif kind == "oxts":
                parts = path.read_text(encoding="utf-8").split()
                if len(parts) < 30:
                    errors.append("OXTS packet has fewer than 30 fields: " + str(path))
                else:
                    for entry in parts[:6]:
                        float(entry)
        except (OSError, ValueError) as exc:
            errors.append("Unreadable sample " + str(path) + ": " + str(exc))


def audit_sequence(sequence: str | Path) -> dict:
    seq = Path(sequence).expanduser().resolve()
    result: dict = {
        "sequence": str(seq),
        "note": "sync OXTS samples are NOT a complete high-rate raw IMU stream",
        "streams": {},
        "calibration": {},
        "alignment": {},
        "errors": [],
        "warnings": [],
    }
    errors: list[str] = result["errors"]
    if not seq.is_dir():
        errors.append("Sequence directory does not exist: " + str(seq))
        result["status"] = "FAIL"
        return result

    ids_by_stream: dict[str, list[str]] = {}
    times_by_stream: dict[str, list[int]] = {}
    for kind, extension in STREAMS.items():
        data_dir = seq / kind / "data"
        files = sorted(data_dir.glob("*" + extension)) if data_dir.is_dir() else []
        times = _read_times(seq / kind / "timestamps.txt", errors)
        ids = [p.stem for p in files]
        ids_by_stream[kind] = ids
        times_by_stream[kind] = times
        if not files:
            errors.append("No " + extension + " files in " + str(data_dir))
        if len(files) != len(times):
            errors.append(f"{kind}: files={len(files)} timestamps={len(times)}")
        if any(not (stem.isdigit() and len(stem) == 10) for stem in ids):
            errors.append(kind + ": unexpected frame naming")
        rate = None
        if len(times) >= 2:
            interval = statistics.median((b - a) / 1e9
                                         for a, b in zip(times, times[1:]))
            rate = round(1 / interval, 3) if interval > 0 else None
        result["streams"][kind] = {
            "frames": len(files), "timestamps": len(times),
            "median_rate_hz": rate,
            "first_frame": ids[0] if ids else None,
            "last_frame": ids[-1] if ids else None,
        }
        _check_samples(files, kind, errors)

    ref_ids = ids_by_stream["oxts"]
    ref_times = times_by_stream["oxts"]
    for kind in STREAMS:
        ids = ids_by_stream[kind]
        result["alignment"][kind] = {"same_frame_ids_as_oxts": ids == ref_ids}
        if ids != ref_ids:
            errors.append("Frame ID mismatch: oxts vs " + kind)
        ts = times_by_stream[kind]
        if ref_times and len(ts) == len(ref_times):
            diffs = [abs(a - b) / 1e6 for a, b in zip(ts, ref_times)]
            result["alignment"][kind]["max_timestamp_offset_ms"] = round(
                max(diffs), 6
            ) if diffs else 0.0

    for name in CALIB_FILES:
        path = seq.parent / name
        result["calibration"][name] = path.is_file() and path.stat().st_size > 0
        if not result["calibration"][name]:
            errors.append("Missing calibration: " + str(path))

    if ref_times and len(ref_times) < 10:
        result["warnings"].append("Very short sequence: less than 10 frames")
    result["status"] = "PASS" if not errors else "FAIL"
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sequence", required=True, help="KITTI *_sync drive directory")
    parser.add_argument("--report", help="Optional JSON report destination")
    args = parser.parse_args()
    result = audit_sequence(args.sequence)
    output = json.dumps(result, ensure_ascii=False, indent=2)
    print(output)
    if args.report:
        report_path = Path(args.report).expanduser()
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(output + "\n", encoding="utf-8")
    return 0 if result["status"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
