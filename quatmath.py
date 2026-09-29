"""Quaternion math and robust statistics.

Quaternions are (w, x, y, z) tuples in SlimeVR's frame (Y up).

Yaw is measured as the *twist around the world vertical axis* of the delta
rotation between two orientations. Unlike extracting Euler angles, this
stays well defined with the tracker lying down, on its side or upside down.
"""

import math


def mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )


def conj(q):
    return (q[0], -q[1], -q[2], -q[3])


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def world_yaw_delta_deg(q_from, q_to):
    """How much the orientation turned around the vertical axis (deg, -180..180).

    Gyroscope drift shows up as a world-frame rotation applied on the left
    (q_to = R_y(d) * q_from), so the world delta is q_to * conj(q_from) and
    the yaw is its twist around Y.
    """
    d = mul(q_to, conj(q_from))
    return wrap_deg(math.degrees(2.0 * math.atan2(d[2], d[0])))


def angle_deg(a, b):
    """Total angle between two orientations (degrees)."""
    dot = abs(sum(x * y for x, y in zip(a, b)))
    return math.degrees(2.0 * math.acos(min(1.0, dot)))


def median(values):
    s = sorted(values)
    n = len(s)
    if n == 0:
        return float("nan")
    mid = n // 2
    return s[mid] if n % 2 else (s[mid - 1] + s[mid]) / 2


def mad(values):
    """Median absolute deviation: robust against isolated spikes."""
    if not values:
        return float("nan")
    m = median(values)
    return median([abs(v - m) for v in values])


def slope(ts, ys):
    """Least-squares slope (units of y per second)."""
    n = len(ts)
    if n < 2:
        return float("nan")
    mt, my = sum(ts) / n, sum(ys) / n
    den = sum((t - mt) ** 2 for t in ts)
    if den == 0:
        return float("nan")
    return sum((t - mt) * (y - my) for t, y in zip(ts, ys)) / den


def percentile(values, p):
    s = sorted(values)
    if not s:
        return float("nan")
    k = (len(s) - 1) * p / 100.0
    lo, hi = math.floor(k), math.ceil(k)
    return s[lo] + (s[hi] - s[lo]) * (k - lo)
