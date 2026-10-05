"""2D calibration transform math for the operator UI world frame.

Extracted from the HTTP handler so it can be unit-tested in isolation
(see client/tests/test_transform.py). The behaviour mirrors the original
inline calibration in server/server.py exactly.

Model: ``world = R @ local + T`` where ``R`` is a row-major 2x2 matrix and
``T`` a 2-vector. A 3-point calibration defines the world frame:

    A -> origin, B -> +x direction, C -> +y direction

These are drone-local poses captured by the operator during calibration.
NOTE: the original calibration is intentionally *naive* (no orthonormalisation
of the C axis); this module preserves that so the tests characterise real
behaviour. The axis convention here is what guards against the known
"rover goes right instead of left" inversion bug (roadmap item 2).
"""
from __future__ import annotations

import math


def compute_calibration(A, B, C):
    """Compute ``(R, T)`` from three drone-local calibration points.

    ``A``, ``B``, ``C`` are ``(x, y, ...)`` tuples (extra components ignored).
    Returns ``(R, T)`` such that ``apply_transform(R, T, A) == (0.0, 0.0)``.

    Raises ``ValueError`` if A->B or A->C has zero length (degenerate
    calibration), instead of the original ``ZeroDivisionError``.
    """
    Ax, Ay = A[0], A[1]
    Bx, By = B[0], B[1]
    Cx, Cy = C[0], C[1]

    # x-axis: unit vector from A to B
    vx = (Bx - Ax, By - Ay)
    lnx = math.hypot(*vx)
    if lnx == 0.0:
        raise ValueError("Degenerate calibration: A and B coincide")
    ux = (vx[0] / lnx, vx[1] / lnx)

    # y-axis: unit vector from A to C
    vy = (Cx - Ax, Cy - Ay)
    lny = math.hypot(*vy)
    if lny == 0.0:
        raise ValueError("Degenerate calibration: A and C coincide")
    uy = (vy[0] / lny, vy[1] / lny)

    # R has the basis vectors as columns (matches the original server.py).
    R = [
        [ux[0], uy[0]],
        [ux[1], uy[1]],
    ]
    # T places the world origin at A: T = -R @ A.
    T = (
        -Ax * R[0][0] - Ay * R[0][1],
        -Ax * R[1][0] - Ay * R[1][1],
    )
    return R, T


def apply_transform(R, T, local):
    """Map a drone-local point to world coordinates: ``world = R @ local + T``."""
    lx, ly = local[0], local[1]
    wx = R[0][0] * lx + R[0][1] * ly + T[0]
    wy = R[1][0] * lx + R[1][1] * ly + T[1]
    return (wx, wy)


def world_to_local(R, T, world):
    """Inverse transform: map a world point back to drone-local coordinates.

    This is the drone->rover direction used when sending nav goals, where the
    axis-inversion bug (roadmap item 2) shows up. ``world_to_local`` is the
    exact inverse of ``apply_transform`` for the same ``(R, T)``.
    """
    det = R[0][0] * R[1][1] - R[0][1] * R[1][0]
    if det == 0.0:
        raise ValueError("Non-invertible transform (det == 0)")

    ax = world[0] - T[0]
    ay = world[1] - T[1]

    # Inverse of a 2x2 matrix.
    i00 = R[1][1] / det
    i01 = -R[0][1] / det
    i10 = -R[1][0] / det
    i11 = R[0][0] / det

    lx = i00 * ax + i01 * ay
    ly = i10 * ax + i11 * ay
    return (lx, ly)
