"""Fixed-noise KITTI OXTS sparse-IMU + position factor GTSAM demo.

This is NOT an independent localization benchmark. KITTI synced OXTS
(~10 Hz) is not the original high-rate IMU stream. GPS positions and
initial navigation state come from the same OXTS unit. Only the first
OXTS velocity and attitude are permitted for initialization; later
velocity/attitude are intentionally excluded.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

from scripts.audit_oxts import read_sequence_oxts
from src.coordinates import geodetic_to_enu


@dataclass(frozen=True)
class Sample:
    time_ns: int
    position_enu_m: tuple[float, float, float]
    accel_body_mps2: tuple[float, float, float]
    gyro_body_radps: tuple[float, float, float]
    init_velocity_enu_mps: tuple[float, float, float]
    init_yaw_pitch_roll_rad: tuple[float, float, float]


@dataclass(frozen=True)
class FixedNoise:
    gravity_mps2: float = 9.81
    accel_density: float = 0.2
    gyro_density: float = 0.02
    gps_sigma_xy: float = 3.0
    gps_sigma_z: float = 5.0
    pose_prior_rot_sigma: float = 0.1
    pose_prior_pos_sigma: float = 1.0
    velocity_prior_sigma: float = 1.0
    accel_bias_prior_sigma: float = 0.5
    gyro_bias_prior_sigma: float = 0.05
    accel_bias_rw: float = 0.02
    gyro_bias_rw: float = 0.002

    def validate(self) -> None:
        for name, value in vars(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")


def load_samples(sequence: Path, max_frames: int) -> list[Sample]:
    if max_frames < 2:
        raise ValueError("max_frames must be >= 2")
    packets, times = read_sequence_oxts(sequence)
    packets, times = packets[:max_frames], times[:max_frames]
    if len(packets) < 2:
        raise ValueError("Need at least two synchronized OXTS samples")
    origin = tuple(packets[0][k] for k in ("lat", "lon", "alt"))
    samples = []
    for p, time in zip(packets, times):
        samples.append(Sample(
            time_ns=time,
            position_enu_m=geodetic_to_enu(p["lat"], p["lon"], p["alt"], *origin),
            accel_body_mps2=(p["ax"], p["ay"], p["az"]),
            gyro_body_radps=(p["wx"], p["wy"], p["wz"]),
            init_velocity_enu_mps=(p["ve"], p["vn"], p["vu"]),
            init_yaw_pitch_roll_rad=(p["yaw"], p["pitch"], p["roll"]),
        ))
    if any(b.time_ns <= a.time_ns for a, b in zip(samples, samples[1:])):
        raise ValueError("Non-monotonic OXTS times")
    return samples


def run_gtsam(samples: list[Sample], noise: FixedNoise, iterations: int = 30) -> dict:
    if len(samples) < 2 or iterations < 1:
        raise ValueError("At least two frames and positive iterations required")
    noise.validate()
    try:
        import numpy as np
        import gtsam
        from gtsam.symbol_shorthand import X, V, B
    except ImportError as exc:
        raise RuntimeError(
            "Install project dependencies in a dedicated environment: "
            "python -m pip install -r requirements.txt"
        ) from exc

    graph, values = gtsam.NonlinearFactorGraph(), gtsam.Values()
    bias0 = gtsam.imuBias.ConstantBias(np.zeros(3), np.zeros(3))
    first = samples[0]
    yaw, pitch, roll = first.init_yaw_pitch_roll_rad
    pose0 = gtsam.Pose3(gtsam.Rot3.Ypr(yaw, pitch, roll),
                       np.asarray(first.position_enu_m, dtype=float))
    velocity0 = np.asarray(first.init_velocity_enu_mps, dtype=float)
    values.insert(X(0), pose0)
    values.insert(V(0), velocity0)
    values.insert(B(0), bias0)

    graph.add(gtsam.PriorFactorPose3(
        X(0), pose0,
        gtsam.noiseModel.Diagonal.Sigmas(np.array(
            [noise.pose_prior_rot_sigma] * 3
            + [noise.pose_prior_pos_sigma] * 3
        ))
    ))
    graph.add(gtsam.PriorFactorVector(
        V(0), velocity0,
        gtsam.noiseModel.Isotropic.Sigma(3, noise.velocity_prior_sigma)
    ))
    graph.add(gtsam.PriorFactorConstantBias(
        B(0), bias0,
        gtsam.noiseModel.Diagonal.Sigmas(np.array(
            [noise.accel_bias_prior_sigma] * 3
            + [noise.gyro_bias_prior_sigma] * 3
        ))
    ))
    gps_noise = gtsam.noiseModel.Diagonal.Sigmas(
        np.array([noise.gps_sigma_xy, noise.gps_sigma_xy, noise.gps_sigma_z])
    )
    graph.add(gtsam.GPSFactor(
        X(0), np.asarray(first.position_enu_m), gps_noise
    ))

    # GTSAM MakeSharedU is ENU, gravity [0, 0, -9.81].
    params = gtsam.PreintegrationParams.MakeSharedU(noise.gravity_mps2)
    params.setAccelerometerCovariance(np.eye(3) * noise.accel_density ** 2)
    params.setGyroscopeCovariance(np.eye(3) * noise.gyro_density ** 2)
    params.setIntegrationCovariance(np.eye(3) * 1e-8)

    propagated = gtsam.NavState(pose0, velocity0)
    for index in range(1, len(samples)):
        previous, current = samples[index - 1:index + 1]
        dt = (current.time_ns - previous.time_ns) / 1e9
        if not 0.0 < dt < 1.0:
            raise ValueError(f"Unexpected interval at frame {index}: {dt}")
        pim = gtsam.PreintegratedImuMeasurements(params, bias0)
        # Zero-order hold of sparse IMU for an engineering smoke test only.
        pim.integrateMeasurement(
            np.asarray(previous.accel_body_mps2),
            np.asarray(previous.gyro_body_radps), dt
        )
        graph.add(gtsam.ImuFactor(
            X(index - 1), V(index - 1), X(index), V(index), B(index - 1), pim
        ))
        bias_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array(
            [noise.accel_bias_rw * math.sqrt(dt)] * 3
            + [noise.gyro_bias_rw * math.sqrt(dt)] * 3
        ))
        graph.add(gtsam.BetweenFactorConstantBias(
            B(index - 1), B(index), bias0, bias_noise
        ))
        graph.add(gtsam.GPSFactor(
            X(index), np.asarray(current.position_enu_m), gps_noise
        ))
        propagated = pim.predict(propagated, bias0)
        values.insert(X(index), propagated.pose())
        values.insert(V(index), propagated.velocity())
        values.insert(B(index), bias0)

    before = float(graph.error(values))
    opt_params = gtsam.LevenbergMarquardtParams()
    opt_params.setMaxIterations(iterations)
    result = gtsam.LevenbergMarquardtOptimizer(
        graph, values, opt_params
    ).optimize()
    after = float(graph.error(result))
    if not math.isfinite(after):
        raise RuntimeError("Non-finite GTSAM objective")
    # Signed per-axis discrepancy = estimated state minus INPUT OXTS fix.
    # Unlike independent ATE, this is strictly an in-sample measurement fit.
    target = np.array([sample.position_enu_m for sample in samples], dtype=float)
    initialized = np.array([
        values.atPose3(X(i)).translation() for i in range(len(samples))
    ], dtype=float)
    estimated = np.array([
        result.atPose3(X(i)).translation() for i in range(len(samples))
    ], dtype=float)
    differences = estimated - target
    residual_norms = np.linalg.norm(differences, axis=1)
    initial_norms = np.linalg.norm(initialized - target, axis=1)
    axis_rmse = np.sqrt(np.mean(differences ** 2, axis=0))
    axis_mean = np.mean(differences, axis=0)
    worst_frame = int(np.argmax(residual_norms))
    total_rmse = float(np.sqrt(np.mean(residual_norms ** 2)))
    diagnostics = []
    if total_rmse > noise.gps_sigma_xy:
        diagnostics.append(
            "OXTS position fit RMSE exceeds configured horizontal GPS sigma; "
            "inspect component residuals, sparse IMU compatibility, and initialization"
        )
    if after > before + 1e-6:
        diagnostics.append("Optimizer objective increased; inspect convergence")

    return {
        "frames": len(samples),
        "factor_count": int(graph.size()),
        "initial_objective": before,
        "optimized_objective": after,
        "oxts_position_fit_rmse_m": total_rmse,
        "initial_oxts_position_fit_rmse_m": float(
            np.sqrt(np.mean(initial_norms ** 2))
        ),
        "oxts_position_fit_axis_rmse_m": {
            axis: float(axis_rmse[i]) for i, axis in enumerate("ENU")
        },
        "oxts_position_fit_axis_mean_m": {
            axis: float(axis_mean[i]) for i, axis in enumerate("ENU")
        },
        "oxts_position_fit_median_m": float(np.median(residual_norms)),
        "oxts_position_fit_p95_m": float(np.percentile(residual_norms, 95)),
        "oxts_position_fit_max_m": float(residual_norms[worst_frame]),
        "oxts_position_fit_max_frame": worst_frame,
        "oxts_position_fit_first_frame_m": float(residual_norms[0]),
        "oxts_position_fit_last_frame_m": float(residual_norms[-1]),
        "last_oxts_input_enu_m": [float(v) for v in target[-1]],
        "last_estimated_enu_m": [float(v) for v in estimated[-1]],
        "diagnostic_warnings": diagnostics,
        "description": "GTSAM GPSFactor/ImuFactor + random-walk IMU bias",
        "caution": (
            "Uses 10 Hz synchronized OXTS position/acceleration/angular rate. "
            "First OXTS orientation and velocity are only initial conditions. "
            "Position fit RMSE is NOT independent localization ATE."
        ),
    }
