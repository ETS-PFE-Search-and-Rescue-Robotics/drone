"""Contract tests for the drone<->rover UDP packet formats (protocol.py).

Guards the fragile, schema-less UDP link: if the ``ROVER,x,y,o`` format is
changed on the drone side, these fail. Keep in sync with the rover decoder
in robot / ugv_ws. Pure stdlib — runs under pytest and directly.
"""
import os
import sys

_CLIENT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _CLIENT_DIR not in sys.path:
    sys.path.insert(0, _CLIENT_DIR)

from drone_infos.protocol import (  # noqa: E402
    ROVER_COMMAND_PORT,
    ROVER_TELEMETRY_PORT,
    encode_rover_telemetry,
    parse_rover_telemetry,
)


def test_ports_match_architecture_doc():
    # Documented in org-site/docs/architecture.md and CLAUDE.md.
    assert ROVER_COMMAND_PORT == 5005
    assert ROVER_TELEMETRY_PORT == 5006


def test_telemetry_roundtrip():
    msg = encode_rover_telemetry(1.5, -2.25, 0.75)
    assert msg.startswith("ROVER,")
    assert parse_rover_telemetry(msg) == (1.5, -2.25, 0.75)


def test_parse_accepts_integer_like_fields():
    assert parse_rover_telemetry("ROVER,1,2,3") == (1.0, 2.0, 3.0)


def test_parse_tolerates_whitespace():
    assert parse_rover_telemetry("  ROVER,0,0,0\n") == (0.0, 0.0, 0.0)


def test_parse_rejects_wrong_prefix():
    assert parse_rover_telemetry("DRONE,1,2,3") is None


def test_parse_rejects_short_message():
    assert parse_rover_telemetry("ROVER,1,2") is None


def test_parse_rejects_non_numeric():
    assert parse_rover_telemetry("ROVER,x,y,z") is None


def test_parse_handles_none_and_empty():
    assert parse_rover_telemetry(None) is None
    assert parse_rover_telemetry("") is None


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
