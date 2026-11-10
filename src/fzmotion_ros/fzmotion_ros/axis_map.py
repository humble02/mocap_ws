"""Pure rotation helpers: map FZMotion (Y-up) vectors / orientations to ROS (Z-up).

Unified mapping (theta = 前方相对 FZ 世界水平方向的夹角, radians):

    p_ros = Rz(theta) * Rx(+90deg) * p_fz
    R_ros = Rz(theta) * Rx(+90deg) * R_fz * (Rz(theta) * Rx(+90deg))^T

  theta = 0      -> ros = ( x, -z,  y)   (情况 A: FZ +X 为前)
  theta = +90deg -> ros = ( z,  x,  y)   (情况 B: FZ +Z 为前)
  theta = -90deg -> ros = (-z, -x,  y)   (情况 C: FZ -Z 为前)

No ROS / protobuf imports here so it can be unit-tested standalone.
"""

import math

_HALF_SQRT2 = math.sqrt(0.5)


def _qmul(a, b):
    """Hamilton product of two quaternions (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def map_quaternion(theta_rad):
    """Quaternion of R_map = Rz(theta) * Rx(+90deg)."""
    q_rz = (0.0, 0.0, math.sin(theta_rad / 2.0), math.cos(theta_rad / 2.0))
    q_rx = (_HALF_SQRT2, 0.0, 0.0, _HALF_SQRT2)   # +90deg about X
    return _qmul(q_rz, q_rx)


def fz_vec_to_ros(v, theta_rad, apply_map=True):
    """Map a 3-vector (position / linear vel / accel / angular) FZ -> ROS."""
    x, y, z = float(v[0]), float(v[1]), float(v[2])
    if not apply_map:
        return (x, y, z)
    c, s = math.cos(theta_rad), math.sin(theta_rad)
    vx, vy, vz = x, -z, y                          # Rx(+90deg)
    return (c * vx - s * vy, s * vx + c * vy, vz)  # Rz(theta)


def fz_quat_to_ros(q, theta_rad, apply_map=True):
    """Map an orientation quaternion (x, y, z, w) FZ -> ROS via similarity transform."""
    if not apply_map:
        return tuple(q)
    qm = map_quaternion(theta_rad)
    qmi = (-qm[0], -qm[1], -qm[2], qm[3])
    x, y, z, w = _qmul(_qmul(qm, q), qmi)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / n, y / n, z / n, w / n)


def quat_rotate(q, v):
    """Rotate a body-frame vector v by quaternion q (body->world): returns R(q)*v.

    Used for FZMotion angular velocity / angular acceleration, which are reported
    in the rigid body's own frame; rotating them by the body orientation gives the
    world-frame vector.
    """
    x, y, z, w = q
    vx, vy, vz = float(v[0]), float(v[1]), float(v[2])
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )
