"""WGS84 geodetic coordinates -> ECEF -> local ENU (meters).

Conventions:
- input geodetic coordinates: latitude/longitude in degrees, altitude in meters.
- ECEF: right-handed Earth-centered coordinates in meters.
- ENU: right-handed (East, North, Up), tangent at a fixed reference geodetic point.
- This module makes NO claim that OXTS-derived position is independent ground truth.
"""
from __future__ import annotations

import math

WGS84_A_M = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


def geodetic_to_ecef(lat_deg: float, lon_deg: float, altitude_m: float) -> tuple[float, float, float]:
    """Convert WGS84 geodetic latitude, longitude and ellipsoidal height to ECEF."""
    lat, lon, h = float(lat_deg), float(lon_deg), float(altitude_m)
    if not all(math.isfinite(v) for v in (lat, lon, h)):
        raise ValueError("Geodetic coordinates must be finite")
    if not -90.0 <= lat <= 90.0 or not -180.0 <= lon <= 180.0:
        raise ValueError(f"Invalid latitude/longitude: {lat}, {lon}")
    phi, lam = math.radians(lat), math.radians(lon)
    sin_phi = math.sin(phi)
    n = WGS84_A_M / math.sqrt(1.0 - WGS84_E2 * sin_phi * sin_phi)
    return (
        (n + h) * math.cos(phi) * math.cos(lam),
        (n + h) * math.cos(phi) * math.sin(lam),
        (n * (1.0 - WGS84_E2) + h) * sin_phi,
    )


def ecef_to_enu(
    x_m: float, y_m: float, z_m: float,
    ref_lat_deg: float, ref_lon_deg: float, ref_alt_m: float,
) -> tuple[float, float, float]:
    """Rotate ECEF displacement into an ENU frame fixed at the reference."""
    x, y, z = float(x_m), float(y_m), float(z_m)
    if not all(math.isfinite(v) for v in (x, y, z)):
        raise ValueError("ECEF coordinates must be finite")
    x0, y0, z0 = geodetic_to_ecef(ref_lat_deg, ref_lon_deg, ref_alt_m)
    phi, lam = math.radians(ref_lat_deg), math.radians(ref_lon_deg)
    sp, cp = math.sin(phi), math.cos(phi)
    sl, cl = math.sin(lam), math.cos(lam)
    dx, dy, dz = x - x0, y - y0, z - z0
    return (
        -sl * dx + cl * dy,
        -sp * cl * dx - sp * sl * dy + cp * dz,
        cp * cl * dx + cp * sl * dy + sp * dz,
    )


def geodetic_to_enu(
    lat_deg: float, lon_deg: float, altitude_m: float,
    ref_lat_deg: float, ref_lon_deg: float, ref_alt_m: float,
) -> tuple[float, float, float]:
    """Convert a WGS84 fix directly to a reference-fixed ENU position in meters."""
    x, y, z = geodetic_to_ecef(lat_deg, lon_deg, altitude_m)
    return ecef_to_enu(x, y, z, ref_lat_deg, ref_lon_deg, ref_alt_m)
