#!/usr/bin/env python3
"""Stage 1A: verify OXTS WGS84 -> ECEF -> ENU coordinates and motion consistency.

This is a same-source OXTS sanity check, NOT independent localization accuracy.
No deep learning, GTSAM, plotting dependencies, or fixed-dt assumptions.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts.audit_oxts import SEQUENCES, read_sequence_oxts, stats
from src.coordinates import geodetic_to_enu


def analyze_sequence(sequence: Path) -> dict:
    packets, timestamps_ns = read_sequence_oxts(sequence)
    first = packets[0]
    origin = (first["lat"], first["lon"], first["alt"])

    enu = [
        geodetic_to_enu(p["lat"], p["lon"], p["alt"], *origin)
        for p in packets
    ]
    if any(not math.isfinite(v) for point in enu for v in point):
        raise ValueError(f"{sequence.name}: ENU coordinates contain NaN or Inf")
    if math.dist(enu[0], (0.0, 0.0, 0.0)) > 1e-6:
        raise ValueError(f"{sequence.name}: first ENU position is not the origin")

    # OXTS "ve", "vn", "vu" are the same-source navigation velocities.
    # Differences quantify internal consistency, never independent accuracy.
    horizontal_errors = []
    vertical_errors = []
    implied_horizontal_speeds = []
    time_steps = []
    for i in range(1, len(enu)):
        dt_s = (timestamps_ns[i] - timestamps_ns[i - 1]) / 1e9
        if dt_s <= 0.0:
            raise ValueError(f"{sequence.name}: invalid time step at {i}")
        delta = tuple((enu[i][j] - enu[i - 1][j]) / dt_s for j in range(3))
        expected = tuple((packets[i - 1][key] + packets[i][key]) / 2.0
                         for key in ("ve", "vn", "vu"))
        horizontal_errors.append(math.hypot(delta[0] - expected[0],
                                            delta[1] - expected[1]))
        vertical_errors.append(abs(delta[2] - expected[2]))
        implied_horizontal_speeds.append(math.hypot(delta[0], delta[1]))
        time_steps.append(dt_s)

    path_length_m = sum(math.dist(a, b) for a, b in zip(enu, enu[1:]))
    return {
        "sequence": sequence.name,
        "frames": len(packets),
        "coordinate_system": "local tangent ENU, meters, per-sequence origin",
        "source": "KITTI Raw synchronized OXTS (not independent GNSS ground truth)",
        "origin_wgs84_lat_lon_alt": list(origin),
        "first_enu_m": list(enu[0]),
        "last_enu_m": [round(v, 6) for v in enu[-1]],
        "horizontal_endpoint_displacement_m": round(math.hypot(
            enu[-1][0], enu[-1][1]), 6),
        "full_3d_path_length_m": round(path_length_m, 6),
        "timestamp_delta_s": stats(time_steps),
        "derived_horizontal_speed_mps": stats(implied_horizontal_speeds),
        "oxts_horizontal_velocity_difference_mps": stats(horizontal_errors),
        "oxts_vertical_velocity_difference_mps": stats(vertical_errors),
        "cautions": [
            "OXTS-derived position and velocity have shared navigation source.",
            "All sequences use their own first OXTS fix as an ENU origin.",
            "Altitude is used as an ellipsoidal-height approximation pending datum validation.",
            "No IMU preintegration or independent accuracy measurement is performed.",
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", default="dataset/kitti/2011_10_03",
                        help="KITTI Raw date folder")
    parser.add_argument("--sequences", nargs="+", default=list(SEQUENCES))
    parser.add_argument("--report", default="reports/coordinates_summary.json")
    args = parser.parse_args()

    root = Path(args.root).expanduser().resolve()
    result = {
        "stage": "1A",
        "purpose": "Coordinate transformation and same-source physical consistency",
        "not_independent_ground_truth": True,
        "sequences": {},
        "errors": [],
    }
    for seq_id in args.sequences:
        sequence = root / f"2011_10_03_drive_{seq_id}_sync"
        try:
            item = analyze_sequence(sequence)
            result["sequences"][seq_id] = item
            print(
                f"{seq_id}: frames={item['frames']} "
                f"3D_distance={item['full_3d_path_length_m']:.2f} m "
                f"endpoint_horizontal={item['horizontal_endpoint_displacement_m']:.2f} m"
            )
            v_error = item["oxts_horizontal_velocity_difference_mps"]
            z_error = item["oxts_vertical_velocity_difference_mps"]
            print(f"  OXTS-same-source velocity difference: "
                  f"horizontal median={v_error['median']:.4f}, p95={v_error['p95']:.4f} m/s; "
                  f"vertical p95={z_error['p95']:.4f} m/s")
            print(f"  final ENU (m): {item['last_enu_m']}")
        except (OSError, ValueError, OverflowError) as exc:
            result["errors"].append(f"{seq_id}: {exc}")
            print(f"ERROR {seq_id}: {exc}")

    report = Path(args.report).expanduser()
    report.parent.mkdir(parents=True, exist_ok=True)
    report.write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(f"Report: {report}")
    return 2 if result["errors"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
