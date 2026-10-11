"""Stage 1B tests: sample contract, sparse IMU guard, and optional GTSAM smoke."""
import math
import tempfile
import unittest
from pathlib import Path

from scripts.audit_oxts import FIELDS
from src.gnss_imu import FixedNoise, Sample, load_samples, run_gtsam


def packet_text(lat=49.0, lon=8.0, yaw=0.0, vn=0.0, ve=0.0):
    values = {name: 0 for name in FIELDS}
    values.update({
        "lat": lat, "lon": lon, "alt": 120.0,
        "yaw": yaw, "ve": ve, "vn": vn, "vf": ve,
        "az": 9.81, "pos_accuracy": 1.0, "vel_accuracy": 0.1,
        "numsats": 8, "navstat": 4,
        "posmode": 4, "velmode": 4, "orimode": 4,
    })
    return " ".join(str(values[name]) for name in FIELDS)


class GNSSIMUTests(unittest.TestCase):
    def test_fixed_noise_must_be_positive(self):
        FixedNoise().validate()
        with self.assertRaisesRegex(ValueError, "gps_sigma_xy"):
            FixedNoise(gps_sigma_xy=0).validate()
        with self.assertRaisesRegex(ValueError, "gyro_density"):
            FixedNoise(gyro_density=float("nan")).validate()

    def test_sample_contract(self):
        with tempfile.TemporaryDirectory() as tmp:
            sequence = Path(tmp) / "2011_10_03_drive_0027_sync"
            data_dir = sequence / "oxts" / "data"
            data_dir.mkdir(parents=True)
            for i in range(3):
                (data_dir / f"{i:010d}.txt").write_text(
                    packet_text(lon=8.0 + 0.000001 * i, ve=0.7)
                )
            (sequence / "oxts" / "timestamps.txt").write_text(
                "\n".join(f"2011-10-03 12:00:00.{i}00000000" for i in range(3))
                + "\n"
            )
            samples = load_samples(sequence, max_frames=2)
            self.assertEqual(len(samples), 2)
            self.assertAlmostEqual(samples[0].position_enu_m[0], 0.0, places=6)
            self.assertGreater(samples[1].position_enu_m[0], 0)
            self.assertEqual(samples[0].init_velocity_enu_mps, (0.7, 0.0, 0.0))
            self.assertEqual(samples[0].accel_body_mps2, (0.0, 0.0, 9.81))
            self.assertEqual(
                (samples[1].time_ns - samples[0].time_ns) / 1e9, 0.1
            )

    def test_too_few_frames(self):
        with self.assertRaisesRegex(ValueError, "max_frames"):
            load_samples(Path("/nonexistent"), max_frames=1)

    def test_optional_gtsam_stationary_smoke(self):
        try:
            import gtsam  # noqa: F401
            import numpy  # noqa: F401
        except ImportError:
            self.skipTest("GTSAM not installed; standard-library tests still run")
        epoch_ns = 1_000_000_000_000
        samples = [
            Sample(
                time_ns=epoch_ns + i * 100_000_000,
                position_enu_m=(0.0, 0.0, 0.0),
                accel_body_mps2=(0.0, 0.0, 9.81),
                gyro_body_radps=(0.0, 0.0, 0.0),
                init_velocity_enu_mps=(0.0, 0.0, 0.0),
                init_yaw_pitch_roll_rad=(0.0, 0.0, 0.0),
            )
            for i in range(4)
        ]
        output = run_gtsam(samples, FixedNoise(), iterations=15)
        self.assertLessEqual(output["optimized_objective"],
                             output["initial_objective"] + 1e-5)
        self.assertTrue(math.isfinite(output["oxts_position_fit_rmse_m"]))
        self.assertLess(output["oxts_position_fit_rmse_m"], 0.5)


if __name__ == "__main__":
    unittest.main()
