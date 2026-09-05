"""Phase 1 schema-validation tests for the ros2 component.

Run where ESPHome is installed (it provides esphome.config_validation):
    python -m pytest tests/test_ros2_phase1.py
"""
import importlib.util
import sys
from pathlib import Path

import pytest

COMP_INIT = Path(__file__).parent.parent / "esphome" / "components" / "ros2" / "__init__.py"

esphome = pytest.importorskip("esphome")


def load_ros2_init():
    spec = importlib.util.spec_from_file_location("ros2_under_test", COMP_INIT)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["ros2_under_test"] = mod
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope="module")
def ros2():
    return load_ros2_init()


def _sub(ros2, **kw):
    base = {"topic": "/joint_states", "type": "sensor_msgs/JointState"}
    base.update(kw)
    return ros2.SUBSCRIPTION_SCHEMA(base)


def test_jointstate_requires_targets(ros2):
    with pytest.raises(Exception):
        _sub(ros2, target={"switch": {"id": "x"}})


def test_scalar_rejects_targets(ros2):
    with pytest.raises(Exception):
        ros2.SUBSCRIPTION_SCHEMA(
            {
                "topic": "/gripper/close",
                "type": "std_msgs/Bool",
                "targets": [{"servo": {"id": "s", "joint_name": "j"}}],
            }
        )


def test_duplicate_joint_name_rejected(ros2):
    with pytest.raises(Exception):
        _sub(
            ros2,
            targets=[
                {"servo": {"id": "a", "joint_name": "pan"}},
                {"servo": {"id": "b", "joint_name": "pan"}},
            ],
        )


def test_missing_joint_name_rejected(ros2):
    with pytest.raises(Exception):
        _sub(ros2, targets=[{"servo": {"id": "a"}}])


def test_valid_jointstate_targets(ros2):
    cfg = _sub(
        ros2,
        targets=[
            {"servo": {"id": "a", "joint_name": "pan"}},
            {"servo": {"id": "b", "joint_name": "tilt"}},
        ],
    )
    assert len(cfg["targets"]) == 2


def test_publication_source_exclusivity(ros2):
    with pytest.raises(Exception):
        ros2.PUBLICATION_SCHEMA(
            {
                "topic": "/x",
                "type": "std_msgs/Float32",
                "source": {"sensor": {"id": "s"}},
                "sources": [{"servo": {"id": "a", "joint_name": "j"}}],
            }
        )


def test_supported_types_cover_phase1a(ros2):
    for t in ("std_msgs/Bool", "std_msgs/Float32", "std_msgs/Int32", "std_msgs/String",
              "sensor_msgs/JointState", "trajectory_msgs/JointTrajectory"):
        assert t in ros2.SUPPORTED_TYPES
