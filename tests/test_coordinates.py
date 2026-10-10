"""Stage 1A geometry and OXTS diagnostic tests (no third-party dependencies)."""
import math
import tempfile
import unittest
from pathlib import Path

from scripts.audit_oxts import FIELDS
from scripts.validate_coordinates import analyze_sequence
from src.coordinates import WGS84_A_M, WGS84_F, geodetic_to_ecef, geodetic_to_enu


def _packet(lat: float, lon: float, alt: float = 120.0) -> str:
    values = {name: 0 for name in FIELDS}
    values.update({
        "lat": lat, "lon": lon, "alt": alt,
        "ve": 7.0, "vf": 7.0, "numsats": 8,
        "pos_accuracy": 1.0, "vel_accuracy": 0.2,
        "navstat": 4, "posmode": 4, "velmode": 4, "orimode": 4,
    })
    return " ".join(str(values[name]) for name in FIELDS)


class CoordinateTests(unittest.TestCase):
    def test_ecef_equator_and_pole(self):
        x, y, z = geodetic_to_ecef(0.0, 0.0, 0.0)
        self.assertAlmostEqual(x, WGS84_A_M, places=5)
        self.assertAlmostEqual(y, 0.0, places=5)
        self.assertAlmostEqual(z, 0.0, places=5)
        x, y, z = geodetic_to_ecef(0.0, 90.0, 0.0)
        self.assertAlmostEqual(x, 0.0, places=5)
        self.assertAlmostEqual(y, WGS84_A_M, places=5)
        self.assertAlmostEqual(z, 0.0, places=5)
        x, y, z = geodetic_to_ecef(90.0, 0.0, 0.0)
        expected_b = WGS84_A_M * (1.0 - WGS84_F)
        self.assertAlmostEqual(z, expected_b, places=5)
        self.assertAlmostEqual(x, 0.0, places=5)

    def test_same_reference_is_enu_origin(self):
        e, n, u = geodetic_to_enu(49.0, 8.0, 150.0, 49.0, 8.0, 150.0)
        self.assertLess(math.hypot(e, n, u), 1e-7)

    def test_known_east_and_up_displacement(self):
        # At equator a small longitude change corresponds to approximately
        # WGS84_A_M * delta_longitude_radians meters East.
        lon_deg = math.degrees(1.0 / WGS84_A_M)
        e, n, u = geodetic_to_enu(0.0, lon_deg, 0.0, 0.0, 0.0, 0.0)
        self.assertAlmostEqual(e, 1.0, places=5)
        self.assertAlmostEqual(n, 0.0, places=5)
        self.assertLess(abs(u), 1e-5)
        e, n, u = geodetic_to_enu(0.0, 0.0, 10.0, 0.0, 0.0, 0.0)
        self.assertAlmostEqual(e, 0.0, places=5)
        self.assertAlmostEqual(n, 0.0, places=5)
        self.assertAlmostEqual(u, 10.0, places=5)

    def test_invalid_coordinates_rejected(self):
        for lat, lon, altitude in (
            (91, 0, 0), (0, -181, 0), (float("nan"), 0, 0),
            (0, float("inf"), 0), (0, 0, float("nan")),
        ):
            with self.subTest(lat=lat, lon=lon, altitude=altitude):
                with self.assertRaises(ValueError):
                    geodetic_to_ecef(lat, lon, altitude)

    def test_real_sequence_reader_compatibility(self):
        with tempfile.TemporaryDirectory() as tmp:
            seq = Path(tmp) / "2011_10_03_drive_0027_sync"
            data = seq / "oxts" / "data"
            data.mkdir(parents=True)
            times = []
            for i in range(3):
                # Along East, ~0.73 m per 0.1 s near latitude 49 degrees.
                (data / f"{i:010d}.txt").write_text(
                    _packet(49.0, 8.0 + i * 0.00001) + "\n", encoding="utf-8"
                )
                times.append(f"2011-10-03 12:00:00.{i}00000000")
            (seq / "oxts" / "timestamps.txt").write_text(
                "\n".join(times) + "\n", encoding="utf-8"
            )
            report = analyze_sequence(seq)
            self.assertEqual(report["frames"], 3)
            self.assertLess(math.dist(report["first_enu_m"], (0, 0, 0)), 1e-6)
            self.assertGreater(report["full_3d_path_length_m"], 1.0)
            self.assertTrue(math.isfinite(
                report["oxts_horizontal_velocity_difference_mps"]["p95"]
            ))


if __name__ == "__main__":
    unittest.main()
