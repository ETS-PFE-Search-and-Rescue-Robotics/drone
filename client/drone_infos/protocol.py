"""Canonical drone <-> rover UDP packet formats.

The only live drone<->rover link is UDP, with no schema: changing a packet
format on one side silently breaks the other. This module is the single
source of truth for the drone side of that contract so it can be tested
(see client/tests/test_protocol.py) and kept in sync with the rover.

Wire formats (see org-site/docs/architecture.md):
  * rover -> drone telemetry on UDP :5006 -> ``"ROVER,<x>,<y>,<o>"``
  * drone -> rover commands/pose  on UDP :5005 (plain command string)

If you change anything here, you MUST update the matching encode/decode on
the rover (robot / ugv_ws) or the link breaks.
"""
from __future__ import annotations

ROVER_TELEMETRY_PREFIX = "ROVER"
ROVER_COMMAND_PORT = 5005
ROVER_TELEMETRY_PORT = 5006


def encode_rover_telemetry(x, y, o):
    """Build a ``ROVER,x,y,o`` telemetry string (the rover->drone format)."""
    return "%s,%s,%s,%s" % (ROVER_TELEMETRY_PREFIX, float(x), float(y), float(o))


def parse_rover_telemetry(msg):
    """Parse a ``ROVER,x,y,o`` telemetry string.

    Returns ``(x, y, o)`` as floats, or ``None`` if the message is not valid
    rover telemetry (wrong prefix, too few fields, non-numeric). Never raises.
    """
    if msg is None:
        return None
    parts = msg.strip().split(",")
    if len(parts) < 4 or parts[0] != ROVER_TELEMETRY_PREFIX:
        return None
    try:
        return (float(parts[1]), float(parts[2]), float(parts[3]))
    except ValueError:
        return None
