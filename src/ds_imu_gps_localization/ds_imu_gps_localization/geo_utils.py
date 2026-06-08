"""LLA/ENU helpers using geographiclib (same role as C++ LocalCartesian)."""
import math

import numpy as np
from geographiclib.geodesic import Geodesic

_GEOD = Geodesic.WGS84
DEG2RAD = math.pi / 180.0


def lla_to_enu(lat, lon, alt, lat0, lon0, alt0):
    g = _GEOD.Inverse(lat0, lon0, lat, lon)
    dist = g["s12"]
    az = math.radians(g["azi1"])
    e = dist * math.sin(az)
    n = dist * math.cos(az)
    u = alt - alt0
    return np.array([e, n, u], dtype=float)


def enu_to_lla(e, n, u, lat0, lon0, alt0):
    dist = math.hypot(e, n)
    if dist < 1e-6:
        az = 0.0
    else:
        az = math.degrees(math.atan2(e, n))
    g = _GEOD.Direct(lat0, lon0, az, dist)
    return np.array([g["lat2"], g["lon2"], alt0 + u], dtype=float)


def skew(v):
    x, y, z = v
    return np.array(
        [[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]], dtype=float
    )


def angle_axis_to_rot(angle_axis):
    angle = np.linalg.norm(angle_axis)
    if angle < 1e-12:
        return np.eye(3)
    axis = angle_axis / angle
    k = skew(axis)
    return np.eye(3) + math.sin(angle) * k + (1.0 - math.cos(angle)) * k @ k


def rot_to_quat(rot):
    """Rotation matrix -> quaternion (x, y, z, w)."""
    m = rot
    tr = np.trace(m)
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w], dtype=float)
