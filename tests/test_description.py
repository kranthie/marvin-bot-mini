"""The description check's helpers against hand-worked cases."""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from check_description import bus_cycle_time, expand, main  # noqa: E402

BUS = {
    "baud": 1_000_000,
    "bits_per_byte": 10,
    "usb_latency_s": 0.001,
    "reply_gap_s": 0.0001,
    "write_bytes": 2,
    "read_bytes": 6,
}


def test_bus_cycle_time_six_servos():
    # write 7 + 6 x 3 = 25 bytes, read request 7 + 6 = 13, replies 6 x 12 = 72: 110 bytes at
    # 10 us each = 1.1 ms, two USB round trips 2 ms, six reply gaps 0.6 ms
    assert bus_cycle_time(6, BUS) == pytest.approx(0.0037)


def test_expand_sides():
    out = expand([{"name": "{side}_knee", "drives": ["{side}_knee"]}, {"name": "waist_yaw"}])
    assert [e["name"] for e in out] == ["left_knee", "right_knee", "waist_yaw"]
    assert out[1]["drives"] == ["right_knee"]


def test_description_passes():
    assert main() == 0
