#!/usr/bin/env python3
"""Run Stage 1B GTSAM fixed-noise OXTS engineering demo, NOT paper ATE."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.gnss_imu import FixedNoise, load_samples, run_gtsam


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", default="dataset/kitti/2011_10_03")
    parser.add_argument("--sequence", default="0027", choices=("0027", "0034", "0042", "0047"))
    parser.add_argument("--frames", type=int, default=40)
    parser.add_argument("--iterations", type=int, default=30)
    parser.add_argument("--report", default="reports/gnss_imu_stage1b.json")
    args = parser.parse_args()

    path = Path(args.root).expanduser().resolve() / f"2011_10_03_drive_{args.sequence}_sync"
    samples = load_samples(path, args.frames)
    result = run_gtsam(samples, FixedNoise(), iterations=args.iterations)
    result.update({
        "sequence": args.sequence,
        "stage": "1B",
        "research_status": "engineering smoke test only",
        "parameter_note": "Uncalibrated fixed noise settings; not a tuned reference baseline.",
    })
    report = Path(args.report).expanduser()
    report.parent.mkdir(parents=True, exist_ok=True)
    report.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    print(f"\nReport: {report}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
