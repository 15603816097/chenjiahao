"""Offline tests for Stage-0B timestamp diagnostics."""
import tempfile
import unittest
from pathlib import Path

from scripts.analyze_timing import analyze_sequence, offset_stats, percentile


class TimingTests(unittest.TestCase):
    def test_percentile(self):
        self.assertEqual(percentile([1, 2, 3, 4, 5], 95), 5)
        self.assertEqual(percentile([5, 1, 3], 50), 3)

    def test_offset_sign_and_precision(self):
        ref = [1_000_000_000, 1_100_000_000]
        tgt = [1_060_000_000, 1_160_000_000]
        stats = offset_stats(ref, tgt)
        self.assertEqual(stats["signed_ms"]["median"], 60.0)
        self.assertEqual(stats["absolute_ms"]["max"], 60.0)

    def test_large_offset_is_review_not_failure(self):
        with tempfile.TemporaryDirectory() as td:
            seq = Path(td) / "2011_10_03_drive_0027_sync"
            offsets = {"oxts": 0, "velodyne_points": 60,
                       "image_02": 10, "image_03": 12}
            for stream, offset in offsets.items():
                directory = seq / stream
                directory.mkdir(parents=True)
                lines = []
                for frame in range(3):
                    millis = 100 * frame + offset
                    lines.append(f"2011-10-03 12:00:00.{millis:03d}000000")
                (directory / "timestamps.txt").write_text("\n".join(lines) + "\n")
            report = analyze_sequence(seq)
            self.assertEqual(report["frames"], 3)
            self.assertEqual(
                report["streams"]["velodyne_points"]["relative_to_oxts"]["absolute_ms"]["max"],
                60.0,
            )
            self.assertTrue(any("velodyne_points" in flag for flag in report["review_flags"]))
            self.assertEqual(len(report["streams"]["velodyne_points"]["large_offset_frames"]), 3)
            self.assertEqual(report["streams"]["velodyne_points"]["large_offset_frames"][0]["signed_offset_ms"], 60.0)
            self.assertAlmostEqual(
                report["stereo_image_03_minus_image_02"]["signed_ms"]["median"], 2.0
            )


if __name__ == "__main__":
    unittest.main()
