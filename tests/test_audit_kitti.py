"""Offline tests; no KITTI download or third-party packages required."""
import tempfile
import unittest
from pathlib import Path

from scripts.audit_kitti import audit_sequence, timestamp_ns


class KittiAuditTests(unittest.TestCase):
    def test_nanosecond_precision(self):
        a = timestamp_ns("2011-10-03 12:55:34.997992704")
        b = timestamp_ns("2011-10-03 12:55:34.997992705")
        self.assertEqual(b - a, 1)

    def _sample_sequence(self, folder):
        parent = Path(folder) / "2011_10_03"
        seq = parent / "2011_10_03_drive_0027_sync"
        for name in ("calib_cam_to_cam.txt", "calib_imu_to_velo.txt",
                     "calib_velo_to_cam.txt"):
            (parent / name).parent.mkdir(parents=True, exist_ok=True)
            (parent / name).write_text("R: 1 0 0 0 1 0 0 0 1\n")
        for stream, extension in (
            ("oxts", "txt"),
            ("velodyne_points", "bin"),
            ("image_02", "png"),
            ("image_03", "png"),
        ):
            data = seq / stream / "data"
            data.mkdir(parents=True)
            stamps = []
            for i in range(3):
                file = data / f"{i:010d}.{extension}"
                if stream == "oxts":
                    file.write_text(" ".join(["1"] * 30) + "\n")
                elif stream == "velodyne_points":
                    file.write_bytes(b"\0" * 16)
                else:
                    file.write_bytes(b"\x89PNG\r\n\x1a\n" + b"x")
                stamps.append(f"2011-10-03 12:00:00.{i}00000000")
            (seq / stream / "timestamps.txt").write_text("\n".join(stamps) + "\n")
        return seq

    def test_valid_small_fixture(self):
        with tempfile.TemporaryDirectory() as tmp:
            seq = self._sample_sequence(tmp)
            result = audit_sequence(seq)
            self.assertEqual(result["status"], "PASS", result["errors"])
            self.assertEqual(result["streams"]["oxts"]["frames"], 3)

    def test_missing_stream_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            seq = self._sample_sequence(tmp)
            (seq / "image_03" / "data" / "0000000001.png").unlink()
            result = audit_sequence(seq)
            self.assertEqual(result["status"], "FAIL")
            self.assertTrue(any("image_03" in item for item in result["errors"]))


if __name__ == "__main__":
    unittest.main()
