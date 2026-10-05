"""Tests for the 3-point calibration transform (drone_infos/transform.py).

These lock the current calibration behaviour and guard roadmap item 2
("add automated axis-inversion tests, quantify the error"). Pure stdlib —
runs under pytest in CI, and directly with ``python3 tests/test_transform.py``.
"""
import math
import os
import sys

# Make the client/ package root importable whether run via pytest or directly.
_CLIENT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _CLIENT_DIR not in sys.path:
    sys.path.insert(0, _CLIENT_DIR)

from drone_infos.transform import (  # noqa: E402
    apply_transform,
    compute_calibration,
    world_to_local,
)

TOL = 1e-9


def _close(a, b, tol=TOL):
    return abs(a[0] - b[0]) <= tol and abs(a[1] - b[1]) <= tol


def test_identity_transform_is_a_noop():
    R = [[1.0, 0.0], [0.0, 1.0]]
    T = (0.0, 0.0)
    assert _close(apply_transform(R, T, (3.0, -2.0)), (3.0, -2.0))


def test_calibration_origin_maps_to_zero():
    # A is the world origin by construction, regardless of orientation.
    A, B, C = (1.0, 1.0), (2.0, 1.0), (1.0, 2.0)
    R, T = compute_calibration(A, B, C)
    assert _close(apply_transform(R, T, A), (0.0, 0.0))


def test_axis_aligned_calibration_keeps_axes():
    # Standard axis-aligned calibration: B is +x, C is +y of A.
    # The B direction must land on the world +x axis and C on +y.
    A, B, C = (1.0, 1.0), (2.0, 1.0), (1.0, 2.0)
    R, T = compute_calibration(A, B, C)
    assert _close(apply_transform(R, T, B), (1.0, 0.0))
    assert _close(apply_transform(R, T, C), (0.0, 1.0))


def test_axis_not_inverted():
    # Regression guard for the "rover goes right instead of left" bug:
    # a point in the A->B (right) direction must have a POSITIVE world-x and
    # near-zero world-y; a point in the A->C (forward) direction the opposite.
    A, B, C = (0.0, 0.0), (3.0, 0.0), (0.0, 3.0)
    R, T = compute_calibration(A, B, C)

    right = apply_transform(R, T, (1.0, 0.0))
    assert right[0] > 0.0 and abs(right[1]) < TOL

    forward = apply_transform(R, T, (0.0, 1.0))
    assert forward[1] > 0.0 and abs(forward[0]) < TOL


def test_roundtrip_world_to_local_is_inverse():
    # world_to_local must exactly invert apply_transform for the same (R, T).
    A, B, C = (0.5, -0.5), (1.9, 0.3), (-0.2, 1.4)
    R, T = compute_calibration(A, B, C)
    for p in [(0.0, 0.0), (1.0, 2.0), (-3.3, 4.1), (10.0, -7.5)]:
        world = apply_transform(R, T, p)
        back = world_to_local(R, T, world)
        assert _close(back, p, tol=1e-6)


def test_rotated_calibration_is_a_rotation():
    # A 90-degree-rotated frame: A->B points +y in local space.
    A, B, C = (0.0, 0.0), (0.0, 1.0), (-1.0, 0.0)
    R, T = compute_calibration(A, B, C)
    # B lies on world +x axis (distance 1 from origin).
    bx, by = apply_transform(R, T, B)
    assert math.isclose(math.hypot(bx, by), 1.0, abs_tol=1e-9)


def test_degenerate_calibration_raises():
    try:
        compute_calibration((0.0, 0.0), (0.0, 0.0), (0.0, 1.0))
    except ValueError:
        pass
    else:
        raise AssertionError("expected ValueError for coincident A/B")


# --- Direct-run harness (no pytest required) -------------------------------
if __name__ == "__main__":
    failures = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print(f"PASS {name}")
            except Exception as exc:  # noqa: BLE001
                failures += 1
                print(f"FAIL {name}: {exc!r}")
    print(f"\n{failures} failure(s)")
    sys.exit(1 if failures else 0)
