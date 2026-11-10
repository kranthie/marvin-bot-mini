"""The servo model against the datasheet figures it was derived from."""

import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from sim.servo_model import ServoParams, joint_servos, servo_torque, stall_torque  # noqa: E402

ACTS = yaml.safe_load((ROOT / "description" / "actuators.yaml").read_text())


@pytest.mark.parametrize("model", ["STS3250", "STS3215-C018"])
def test_stall_and_no_load_speed(model):
    a = ACTS[model]
    p = ServoParams.from_actuator(a)
    v = a["v_rated"]
    # a large error saturates the duty: the datasheet's stall torque at rest ...
    assert servo_torque(p, 1.0, 0.0, 0.0, v) == pytest.approx(a["stall_torque"], rel=2e-3)
    assert stall_torque(p, v) == pytest.approx(a["stall_torque"], rel=2e-3)
    # ... and no torque at the datasheet's no-load speed
    w0 = a["no_load_speed_rpm"] * 2 * np.pi / 60
    assert servo_torque(p, 1.0, 0.0, w0, v) == pytest.approx(0.0, abs=0.01)


def test_torque_scales_with_voltage_and_is_odd():
    p = ServoParams.from_actuator(ACTS["STS3250"])
    assert servo_torque(p, 1.0, 0.0, 0.0, 10.5) / servo_torque(p, 1.0, 0.0, 0.0, 12.0) == (
        pytest.approx(10.5 / 12.0)
    )
    assert servo_torque(p, -0.01, 0.0, 0.0, 12.0) == pytest.approx(
        -servo_torque(p, 0.01, 0.0, 0.0, 12.0)
    )


def test_vectorised():
    p = ServoParams.from_actuator(ACTS["STS3250"])
    tau = servo_torque(p, np.array([0.0, 0.05, 1.0]), np.zeros(3), np.zeros(3), 12.0)
    assert tau.shape == (3,) and tau[0] == 0.0 and 0 < tau[1] < tau[2]


def test_every_joint_has_a_servo():
    js = joint_servos()
    assert len(js) == 23 and js["left_knee"][0] == "STS3250" and js["waist_yaw"][0] != "STS3250"
