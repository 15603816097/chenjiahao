#!/usr/bin/env python3
"""Stage 0B: inspect actual KITTI Raw sync timing; never shift frames silently."""
from __future__ import annotations

import argparse
import json
import math
import statistics
from pathlib import Path

try:
    from scripts.audit_kitti import timestamp_ns
except ModuleNotFoundError:
    from audit_kitti import timestamp_ns

STREAMS = ("oxts", "velodyne_points", "image_02", "image_03")
DEFAULT_SEQUENCES = ("0027", "0034", "0042", "0047")


def percentile(values: list[float], percentile_value: float) -> float:
    """Nearest-rank percentile; deterministic without third-party dependencies."""
    if not values:
        raise ValueError("percentile on empty sequence")
    ordered = sorted(values)
    index = max(0, min(len(ordered) - 1, int(math.ceil(
        percentile_value / 100.0 * len(ordered))) - 1))
    return ordered[index]


def summary(values: list[float]) -> dict:
    if not values:
        return {"count": 0}
    return {
        "count": len(values),
        "min": round(min(values), 6),
        "median": round(statistics.median(values), 6),
        "p95": round(percentile(values, 95), 6),
        "max": round(max(values), 6),
    }


def read_times(path: Path) -> list[int]:
    if not path.is_file():
        raise FileNotFoundError(path)
    times = [timestamp_ns(line) for line in path.read_text(encoding="utf-8").splitlines()
             if line.strip()]
    if len(times) < 2:
        raise ValueError(f"{path}: fewer than two timestamps")
    if any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError(f"{path}: timestamps are not strictly increasing")
    return times


def offset_stats(reference: list[int], target: list[int]) -> dict:
    """Signed offset = target time - reference time, milliseconds."""
    if len(reference) != len(target):
        raise ValueError(f"Timestamp count mismatch: {len(reference)} vs {len(target)}")
    signed_ms = [(t - r) / 1_000_000.0 for r, t in zip(reference, target)]
    absolute_ms = [abs(x) for x in signed_ms]
    return {
        "signed_ms": summary(signed_ms),
        "absolute_ms": summary(absolute_ms),
        "first_signed_ms": round(signed_ms[0], 6),
        "last_signed_ms": round(signed_ms[-1], 6),
        "offset_span_ms": round(max(signed_ms) - min(signed_ms), 6),
    }


def analyze_sequence(sequence: Path, review_threshold_ms: float = 50.0) -> dict:
    times = {stream: read_times(sequence / stream / "timestamps.txt")
             for stream in STREAMS}
    reference = times["oxts"]
    result = {
        "sequence": str(sequence),
        "frames": len(reference),
        "streams": {},
        "stereo_image_03_minus_image_02": offset_stats(times["image_02"], times["image_03"]),
        "review_flags": [],
    }
    for stream in STREAMS:
        values = times[stream]
        intervals = [(b - a) / 1e6 for a, b in zip(values, values[1:])]
        med = statistics.median(intervals)
        gaps = sum(delta > 1.5 * med for delta in intervals)
        offsets = offset_stats(reference, values)
        result["streams"][stream] = {
            "frame_count": len(values),
            "frame_interval_ms": summary(intervals),
            "gaps_over_1p5x_median": gaps,
            "relative_to_oxts": offsets,
        }
        if stream != "oxts" and offsets["absolute_ms"]["max"] > review_threshold_ms:
            result["review_flags"].append(
                f"{stream}: max timestamp difference from OXTS exceeds {review_threshold_ms} ms; "
                "inspect before choosing time-association/interpolation policy"
            )
        if gaps:
            result["review_flags"].append(f"{stream}: {gaps} long timestamp intervals")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", required=True, help="KITTI date directory, e.g. dataset/kitti/2011_10_03")
    parser.add_argument("--sequences", nargs="+", default=list(DEFAULT_SEQUENCES),
                        help="Four-digit KITTI drive IDs; default 0027 0034 0042 0047")
    parser.add_argument("--report", default="reports/timing_analysis.json")
    args = parser.parse_args()
    root = Path(args.root).expanduser().resolve()
    report = {"rule": "Frame ID match alone does NOT imply identical acquisition time.",
              "offset_sign": "sensor timestamp minus OXTS timestamp",
              "review_threshold_ms": 50.0, "sequences": {}, "errors": []}
    for seq in args.sequences:
        path = root / f"2011_10_03_drive_{seq}_sync"
        try:
            out = analyze_sequence(path)
            report["sequences"][seq] = out
            print(f"\nSequence {seq}, frames={out['frames']}")
            for name, item in out["streams"].items():
                t = item["frame_interval_ms"]
                off = item["relative_to_oxts"]
                print(f"  {name:16s} dt_med={t['median']:8.3f} ms "
                      f"offset_median={off['signed_ms']['median']:9.3f} ms "
                      f"offset_abs_p95={off['absolute_ms']['p95']:8.3f} ms "
                      f"offset_abs_max={off['absolute_ms']['max']:8.3f} ms "
                      f"gaps={item['gaps_over_1p5x_median']}")
            print("  stereo max absolute difference (ms):",
                  out["stereo_image_03_minus_image_02"]["absolute_ms"]["max"])
            for flag in out["review_flags"]:
                print("  REVIEW:", flag)
        except (OSError, ValueError) as exc:
            report["errors"].append(f"{seq}: {exc}")
            print(f"ERROR {seq}: {exc}")
    path = Path(args.report).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"\nReport: {path}")
    return 2 if report["errors"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
