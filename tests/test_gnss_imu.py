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

    def test_optional_gtsam_constant_east_acceleration(self):
        # Dynamic smoke: known eastward acceleration with gravity compensation.
        # Unlike the zero-motion fixture, this exercises the x-axis and dt^2
        # position integration in GTSAM without changing production parameters.
        try:
            import gtsam  # noqa: F401
            import numpy  # noqa: F401
        except ImportError:
            self.skipTest("GTSAM not installed")
        a_east = 0.6
        initial_speed = 1.25
        samples = []
        for i in range(8):
            t = 0.1 * i
            samples.append(Sample(
                time_ns=1_000_000_000 + 100_000_000 * i,
                position_enu_m=(initial_speed * t + 0.5 * a_east * t * t, 0.0, 0.0),
                accel_body_mps2=(a_east, 0.0, 9.81),
                gyro_body_radps=(0.0, 0.0, 0.0),
                init_velocity_enu_mps=(initial_speed + a_east * t, 0.0, 0.0),
                init_yaw_pitch_roll_rad=(0.0, 0.0, 0.0),
            ))
        report = run_gtsam(samples, FixedNoise(), iterations=20)
        self.assertEqual(report["frames"], 8)
        self.assertEqual(report["factor_count"], 25)
        self.assertLess(report["oxts_position_fit_rmse_m"], 1e-4)
        self.assertLess(
            report["kinematic_consistency"]["imu_accel_minus_oxts_dvdt_rmse_mps2"],
            1e-6,
        )
        self.assertLess(
            report["kinematic_consistency"]["imu_gyro_minus_oxts_attitude_rate_rmse_radps"],
            1e-6,
        )
        # Regression for the single-interval split (two-node factor graph).
        tiny = run_gtsam(samples[:2], FixedNoise(), iterations=10)
        self.assertEqual(tiny["frames"], 2)
        self.assertTrue(math.isfinite(
            tiny["kinematic_consistency"]["accel_difference_norm_rmse_first_half_mps2"]
        ))
        self.assertTrue(math.isfinite(
            tiny["kinematic_consistency"]["accel_difference_norm_rmse_second_half_mps2"]
        ))

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
        self.assertEqual(output["frames"], 4)
        self.assertEqual(output["factor_count"], 13)
        self.assertEqual(set(output["oxts_position_fit_axis_rmse_m"]), {"E", "N", "U"})
        self.assertTrue(all(math.isfinite(v) for v in
                            output["oxts_position_fit_axis_rmse_m"].values()))
        self.assertGreaterEqual(output["oxts_position_fit_max_frame"], 0)
        self.assertLess(output["oxts_position_fit_max_frame"], output["frames"])
        self.assertAlmostEqual(output["oxts_position_fit_last_frame_m"], 0.0, delta=0.5)
        self.assertEqual(len(output["last_oxts_input_enu_m"]), 3)
        self.assertEqual(len(output["last_estimated_enu_m"]), 3)
        self.assertIsInstance(output["diagnostic_warnings"], list)
        self.assertIn("kinematic_consistency", output)
        motion = output["kinematic_consistency"]
        self.assertTrue(motion["later_oxts_navigation_used_for_diagnostics_only"])
        self.assertLess(motion["imu_accel_minus_oxts_dvdt_rmse_mps2"], 1e-6)
        self.assertLess(motion["imu_gyro_minus_oxts_attitude_rate_rmse_radps"], 1e-6)
        self.assertEqual(
            set(motion["imu_gyro_minus_oxts_attitude_rate_axis_mean_radps"]),
            {"X", "Y", "Z"},
        )
        self.assertEqual(
            set(motion["imu_gyro_minus_oxts_attitude_rate_axis_rmse_radps"]),
            {"X", "Y", "Z"},
        )
        self.assertTrue(all(
            abs(x) < 1e-6
            for x in motion["imu_gyro_minus_oxts_attitude_rate_axis_rmse_radps"].values()
        ))
        self.assertLess(motion["accel_difference_norm_rmse_first_half_mps2"], 1e-6)
        self.assertLess(motion["accel_difference_norm_rmse_second_half_mps2"], 1e-6)
        self.assertAlmostEqual(output["oxts_position_fit_rmse_first_half_m"], 0.0, delta=0.5)
        self.assertAlmostEqual(output["oxts_position_fit_rmse_second_half_m"], 0.0, delta=0.5)


if __name__ == "__main__":
    unittest.main()
