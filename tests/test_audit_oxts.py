"""Stage 0C tests: independent of KITTI data and third-party Python packages."""
import tempfile
import unittest
from pathlib import Path

from scripts.audit_oxts import (FIELDS, parse_oxts, horizontal_distance_m,
                                sequence_report, rotation_diagnostics, calibration_report)


def packet_text(lat=49.0, lon=8.0):
    values = {
        "lat": lat, "lon": lon, "alt": 120.0,
        "roll": 0.01, "pitch": 0.01, "yaw": 0.0,
        "vn": 0.0, "ve": 7.0, "vf": 7.0, "vl": 0.0, "vu": 0.0,
        "ax": 0.0, "ay": 0.0, "az": 9.81,
        "af": 0.0, "al": 0.0, "au": 0.0,
        "wx": 0.0, "wy": 0.0, "wz": 0.0,
        "wf": 0.0, "wl": 0.0, "wu": 0.0,
        "pos_accuracy": 0.1, "vel_accuracy": 0.1,
        "navstat": 4, "numsats": 10,
        "posmode": 4, "velmode": 4, "orimode": 4,
    }
    return " ".join(str(values[name]) for name in FIELDS)


class OXTSTests(unittest.TestCase):
    def test_read_30_values(self):
        p = parse_oxts(packet_text())
        self.assertEqual(len(p), 30)
        self.assertEqual(p["numsats"], 10)
        self.assertAlmostEqual(p["az"], 9.81)

    def test_invalid_record_rejected(self):
        with self.assertRaises(ValueError):
            parse_oxts("1 2 3")
        with self.assertRaises(ValueError):
            parse_oxts(packet_text(lat=100.0))
        with self.assertRaises(ValueError):
            parse_oxts(packet_text(lon=float("nan")))

    def test_haversine(self):
        p = parse_oxts(packet_text())
        self.assertEqual(horizontal_distance_m(p, p), 0)
        q = parse_oxts(packet_text(lon=8.00001))
        self.assertGreater(horizontal_distance_m(p, q), 0)

    def test_valid_rotation(self):
        diag = rotation_diagnostics([1, 0, 0, 0, 1, 0, 0, 0, 1])
        self.assertEqual(diag["determinant"], 1.0)
        self.assertEqual(diag["row_orthonormality_max_error"], 0)

    def test_synthetic_sequence_and_calibration(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "2011_10_03"
            seq = root / "2011_10_03_drive_0027_sync"
            data = seq / "oxts" / "data"
            data.mkdir(parents=True)
            timestamps = []
            for i in range(3):
                (data / f"{i:010d}.txt").write_text(
                    packet_text(lon=8.0 + i * 0.00001) + "\n"
                )
                timestamps.append(f"2011-10-03 12:00:00.{i}00000000")
            (seq / "oxts" / "timestamps.txt").write_text("\n".join(timestamps) + "\n")
            report = sequence_report(seq)
            self.assertEqual(report["frame_count"], 3)
            self.assertGreater(report["horizontal_path_length_m"], 0)
            self.assertEqual(report["navigation_flags"]["numsats"]["10"], 3)

            for name in ("calib_velo_to_cam.txt", "calib_imu_to_velo.txt"):
                (root / name).write_text("R: 1 0 0 0 1 0 0 0 1\nT: 0 0 0\n")
            p2 = "700 0 600 0 0 700 200 0 0 0 1 0"
            p3 = "700 0 600 -378 0 700 200 0 0 0 1 0"
            (root / "calib_cam_to_cam.txt").write_text(
                f"P_rect_02: {p2}\nP_rect_03: {p3}\n"
            )
            calib = calibration_report(root)
            self.assertEqual(calib["errors"], [])
            self.assertAlmostEqual(calib["rgb_stereo_baseline_m"], 0.54)


if __name__ == "__main__":
    unittest.main()
