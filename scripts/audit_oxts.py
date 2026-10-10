#!/usr/bin/env python3
"""Stage 0C: audit KITTI OXTS packets and camera/IMU/LiDAR calibrations.

Diagnostic only: OXTS position and OXTS velocity are NOT independent sensors
and must not be treated as an independent GNSS ground truth comparison.
"""
from __future__ import annotations

import argparse
import json
import math
import statistics
from collections import Counter
from decimal import Decimal, InvalidOperation
from pathlib import Path

try:
    from scripts.audit_kitti import timestamp_ns
except ModuleNotFoundError:
    from audit_kitti import timestamp_ns

FIELDS = (
    "lat", "lon", "alt", "roll", "pitch", "yaw",
    "vn", "ve", "vf", "vl", "vu",
    "ax", "ay", "az", "af", "al", "au",
    "wx", "wy", "wz", "wf", "wl", "wu",
    "pos_accuracy", "vel_accuracy",
    "navstat", "numsats", "posmode", "velmode", "orimode",
)
FLAG_FIELDS = FIELDS[-5:]
SEQUENCES = ("0027", "0034", "0042", "0047")
EARTH_RADIUS_M = 6371008.8


def percentile(values: list[float], p: float) -> float:
    if not values:
        raise ValueError("No values to summarize")
    v = sorted(values)
    return v[max(0, min(len(v)-1, math.ceil(p * len(v)/100) - 1))]


def stats(values: list[float]) -> dict:
    if not values:
        return {"count": 0}
    return {
        "count": len(values),
        "min": round(min(values), 6),
        "median": round(statistics.median(values), 6),
        "mean": round(statistics.fmean(values), 6),
        "p95": round(percentile(values, 95), 6),
        "max": round(max(values), 6),
    }


def parse_oxts(text: str) -> dict:
    tokens = text.split()
    if len(tokens) != 30:
        raise ValueError(f"Expected 30 OXTS fields, found {len(tokens)}")
    values: dict = {}
    for index, (name, token) in enumerate(zip(FIELDS, tokens)):
        if index < 25:
            value = float(token)
            if not math.isfinite(value):
                raise ValueError(f"{name} is not finite")
        else:
            # KITTI OXTS flag fields are categorical integers, but some
            # sequences serialize them as decimal strings such as
            # "4.00000000000000". Parse exactly: never silently truncate
            # a fractional category with int(float(token)).
            try:
                decimal_value = Decimal(token)
            except InvalidOperation as exc:
                raise ValueError(f"{name} is not a numeric integer: {token!r}") from exc
            if not decimal_value.is_finite() or decimal_value != decimal_value.to_integral_value():
                raise ValueError(f"{name} must be a finite integer: {token!r}")
            value = int(decimal_value)
        values[name] = value
    if not (-90 <= values["lat"] <= 90):
        raise ValueError(f"latitude out of range: {values['lat']}")
    if not (-180 <= values["lon"] <= 180):
        raise ValueError(f"longitude out of range: {values['lon']}")
    if values["numsats"] < 0:
        raise ValueError("negative satellite count")
    return values


def read_sequence_oxts(sequence: Path) -> tuple[list[dict], list[int]]:
    folder = sequence / "oxts"
    files = sorted((folder / "data").glob("*.txt"))
    stamp_path = folder / "timestamps.txt"
    if not stamp_path.is_file():
        raise FileNotFoundError(stamp_path)
    times = [timestamp_ns(line) for line in stamp_path.read_text(encoding="utf-8").splitlines()
             if line.strip()]
    if not files or len(files) != len(times):
        raise ValueError(f"{sequence}: OXTS files={len(files)}, timestamps={len(times)}")
    if any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError(f"{sequence}: non-monotonic OXTS timestamp")
    if [p.stem for p in files] != [f"{i:010d}" for i in range(len(files))]:
        raise ValueError(f"{sequence}: OXTS packet filename indices are not consecutive")
    packets = []
    for file in files:
        try:
            packets.append(parse_oxts(file.read_text(encoding="utf-8")))
        except (OSError, ValueError) as exc:
            raise ValueError(f"{file}: {exc}") from exc
    return packets, times


def horizontal_distance_m(a: dict, b: dict) -> float:
    """Haversine distance on the Earth surface, for relative sanity checks only."""
    a_lat, b_lat = math.radians(a["lat"]), math.radians(b["lat"])
    d_lat = b_lat - a_lat
    d_lon = math.radians(b["lon"] - a["lon"])
    h = (math.sin(d_lat / 2) ** 2
         + math.cos(a_lat) * math.cos(b_lat) * math.sin(d_lon / 2) ** 2)
    return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(min(1.0, max(0.0, h))))


def sequence_report(sequence: Path) -> dict:
    packets, times = read_sequence_oxts(sequence)
    speed_ned = [math.hypot(p["vn"], p["ve"], p["vu"]) for p in packets]
    horizontal_speed = [math.hypot(p["vn"], p["ve"]) for p in packets]
    acceleration = [math.sqrt(p["ax"] ** 2 + p["ay"] ** 2 + p["az"] ** 2)
                    for p in packets]
    angular_rate = [math.sqrt(p["wx"] ** 2 + p["wy"] ** 2 + p["wz"] ** 2)
                    for p in packets]
    geo_speeds, speed_diffs, distances, dt_values = [], [], [], []
    for i in range(1, len(packets)):
        dt = (times[i] - times[i - 1]) / 1e9
        if dt <= 0:
            raise ValueError("nonpositive interval")
        delta = horizontal_distance_m(packets[i - 1], packets[i])
        derived_speed = delta / dt
        measured_speed = (horizontal_speed[i - 1] + horizontal_speed[i]) / 2
        distances.append(delta)
        geo_speeds.append(derived_speed)
        speed_diffs.append(abs(derived_speed - measured_speed))
        dt_values.append(dt)
    flags = {}
    for name in FLAG_FIELDS:
        counts = Counter(p[name] for p in packets)
        flags[name] = {str(k): counts[k] for k in sorted(counts)}
    report = {
        "sequence": str(sequence),
        "frame_count": len(packets),
        "duration_s": round((times[-1] - times[0]) / 1e9, 6),
        "first_position": {k: packets[0][k] for k in ("lat", "lon", "alt")},
        "last_position": {k: packets[-1][k] for k in ("lat", "lon", "alt")},
        "horizontal_path_length_m": round(sum(distances), 3),
        "horizontal_endpoint_displacement_m": round(horizontal_distance_m(packets[0], packets[-1]), 3),
        "dt_s": stats(dt_values),
        "position": {
            "latitude_deg": stats([p["lat"] for p in packets]),
            "longitude_deg": stats([p["lon"] for p in packets]),
            "altitude_m": stats([p["alt"] for p in packets]),
        },
        "attitude_rad": {name: stats([p[name] for p in packets])
                         for name in ("roll", "pitch", "yaw")},
        "speed_mps": stats(speed_ned),
        "horizontal_velocity_mps": stats(horizontal_speed),
        "position_difference_speed_mps": stats(geo_speeds),
        "same_oxts_speed_discrepancy_mps": stats(speed_diffs),
        "acceleration_xyz_mps2": stats(acceleration),
        "angular_rate_xyz_rps": stats(angular_rate),
        "pos_accuracy_m": stats([p["pos_accuracy"] for p in packets]),
        "vel_accuracy_mps": stats([p["vel_accuracy"] for p in packets]),
        "navigation_flags": flags,
        "review_flags": [],
        "interpretation": (
            "OXTS-derived positions and OXTS velocities are from one navigation system. "
            "Speed discrepancy is a self-consistency diagnostic, NOT independent accuracy. "
            "Raw 'sync' OXTS is downsampled from original 100 Hz recordings."
        ),
    }
    flags_out = report["review_flags"]
    if report["same_oxts_speed_discrepancy_mps"]["p95"] > 10:
        flags_out.append("OXTS position-difference speed vs OXTS velocity: p95 >10 m/s; inspect timebase and position quality")
    if report["speed_mps"]["max"] > 90:
        flags_out.append("Speed >90 m/s: inspect values and units")
    if report["angular_rate_xyz_rps"]["max"] > 15:
        flags_out.append("Angular rate >15 rad/s: inspect values and units")
    if report["acceleration_xyz_mps2"]["max"] > 100:
        flags_out.append("Acceleration >100 m/s²: inspect values and units")
    if any(p["pos_accuracy"] < 0 or p["vel_accuracy"] < 0 for p in packets):
        flags_out.append("Negative position or velocity accuracy value(s)")
    return report


def read_calib(path: Path) -> dict[str, list[float]]:
    if not path.is_file():
        raise FileNotFoundError(path)
    fields = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if ":" not in line:
            continue
        name, values = line.split(":", 1)
        try:
            v = [float(item) for item in values.split()]
        except ValueError:
            continue
        if v:
            fields[name.strip()] = v
    return fields


def rotation_diagnostics(flat: list[float]) -> dict:
    if len(flat) != 9 or any(not math.isfinite(x) for x in flat):
        raise ValueError("Rotation matrix must have 9 finite entries")
    r = [flat[i * 3:(i + 1) * 3] for i in range(3)]
    det = (r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1])
           - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0])
           + r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0]))
    residual = max(abs(sum(r[i][k] * r[j][k] for k in range(3))
                       - (1.0 if i == j else 0.0))
                   for i in range(3) for j in range(3))
    return {"determinant": round(det, 8), "row_orthonormality_max_error": round(residual, 8)}


def calibration_report(date_dir: Path) -> dict:
    out = {"files": {}, "rgb_stereo_baseline_m": None, "errors": [], "warnings": []}
    for name in ("calib_imu_to_velo.txt", "calib_velo_to_cam.txt"):
        try:
            d = read_calib(date_dir / name)
            if "R" not in d or "T" not in d or len(d["T"]) != 3:
                raise ValueError("Missing R (9 floats) or T (3 floats)")
            if not all(math.isfinite(x) for x in d["T"]):
                raise ValueError("Translation contains nonfinite values")
            diag = rotation_diagnostics(d["R"])
            out["files"][name] = diag
            if abs(diag["determinant"] - 1) > 0.01 or diag["row_orthonormality_max_error"] > 0.01:
                out["errors"].append(f"{name}: non-rigid rotation matrix {diag}")
        except (OSError, ValueError) as exc:
            out["errors"].append(f"{name}: {exc}")
    try:
        d = read_calib(date_dir / "calib_cam_to_cam.txt")
        ps = {}
        for cam in ("02", "03"):
            key = "P_rect_" + cam
            p = d.get(key)
            if p is None or len(p) != 12 or any(not math.isfinite(x) for x in p):
                raise ValueError(f"{key} must have 12 finite entries")
            if p[0] <= 0 or p[5] <= 0:
                raise ValueError(f"{key} must have positive focal lengths")
            ps[cam] = p
        # For rectified stereo, camera center x = -Tx / fx.
        centers = {cam: -p[3] / p[0] for cam, p in ps.items()}
        baseline = abs(centers["03"] - centers["02"])
        out["rgb_stereo_baseline_m"] = round(baseline, 6)
        if not 0.3 <= baseline <= 0.8:
            out["warnings"].append(f"RGB stereo baseline {baseline:.3f} m outside review range [0.3, 0.8]")
        out["files"]["calib_cam_to_cam.txt"] = {
            "P_rect_02_fx": ps["02"][0], "P_rect_02_fy": ps["02"][5],
            "P_rect_03_fx": ps["03"][0], "P_rect_03_fy": ps["03"][5],
        }
    except (OSError, ValueError) as exc:
        out["errors"].append(f"calib_cam_to_cam.txt: {exc}")
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", required=True, help="KITTI date folder")
    parser.add_argument("--sequences", nargs="+", default=list(SEQUENCES))
    parser.add_argument("--report", default="reports/oxts_audit.json")
    args = parser.parse_args()
    root = Path(args.root).expanduser().resolve()
    result: dict = {
        "stage": "0C",
        "reference_protocol": "OXTS is not an independent positioning ground truth for OXTS-derived GNSS input.",
        "calibration": calibration_report(root), "sequences": {}, "errors": [],
    }
    for seq_id in args.sequences:
        seq = root / f"2011_10_03_drive_{seq_id}_sync"
        try:
            seq_report = sequence_report(seq)
            result["sequences"][seq_id] = seq_report
            print(f"\n{seq_id}: frames={seq_report['frame_count']}, "
                  f"duration={seq_report['duration_s']:.2f} s, "
                  f"distance={seq_report['horizontal_path_length_m']:.1f} m")
            for field in ("speed_mps", "same_oxts_speed_discrepancy_mps",
                          "acceleration_xyz_mps2", "angular_rate_xyz_rps"):
                item = seq_report[field]
                print(f"  {field:36s} median={item['median']:.3f}, "
                      f"p95={item['p95']:.3f}, max={item['max']:.3f}")
            print(f"  num satellites: {seq_report['navigation_flags']['numsats']}")
            for flag in seq_report["review_flags"]:
                print("  REVIEW:", flag)
        except (OSError, ValueError, OverflowError) as exc:
            result["errors"].append(f"{seq_id}: {exc}")
            print(f"ERROR {seq_id}: {exc}")
    calib = result["calibration"]
    print("\nCalibration: RGB stereo baseline =", calib["rgb_stereo_baseline_m"], "m")
    for flag in calib["errors"]:
        print("  CALIB ERROR:", flag)
    for flag in calib["warnings"]:
        print("  CALIB REVIEW:", flag)
    report = Path(args.report)
    report.parent.mkdir(parents=True, exist_ok=True)
    report.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print("\nReport:", report)
    return 2 if result["errors"] or calib["errors"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
