# Testing — drone

Lightweight regression checks for the drone's own code. ROS2 and VOXL hardware
are **not** required to run these.

## What's covered

| Area | Where | Run without ROS2? |
|------|-------|-------------------|
| 3-point calibration transform (axis-inversion guard, roadmap item 2) | `client/tests/test_transform.py` → `client/drone_infos/transform.py` | yes |
| Drone↔rover UDP packet contract (`ROVER,x,y,o`) | `client/tests/test_protocol.py` → `client/drone_infos/protocol.py` | yes |

The ROS2 packages (`PFE_position`, `PFE_camera_data`) keep their ament
flake8/copyright/pep257 lint stubs; those are not part of this suite.

## Run locally

```bash
# From the drone/ repo root, with pytest installed:
pytest

# Or run a single suite with no pytest at all (pure stdlib):
python3 client/tests/test_transform.py
python3 client/tests/test_protocol.py
```

Config lives in `pyproject.toml` (`testpaths = client/tests`, `pythonpath = client`).

## CI

`.github/workflows/ci.yml` runs on every push/PR: real-error flake8 (blocking),
style flake8 (advisory), `compileall` syntax check, and the unit tests above.
The vendored `voxl-portal-v0.7.11/` tree is excluded.

## Adding tests

Put new pure-Python tests in `client/tests/`. If you test code that's currently
embedded in a handler/thread (like the calibration math used to be in
`server/server.py`), extract the logic into a function in `drone_infos/` first,
then import and test it — see `transform.py` / `protocol.py` as templates.
