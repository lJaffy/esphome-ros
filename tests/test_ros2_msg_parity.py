"""Wire-key parity between ros2 component and canonical .msg definitions.

Checks the C++ structs and JSON keys in esphome/components/ros2 against the
upstream message definitions (registered as a submodule, currently under
scratch/common_interfaces pending a move to third_party/).

Run where pytest is available (needs no esphome install, stdlib only):
    python -m pytest tests/test_ros2_msg_parity.py
"""
import re
from pathlib import Path

import pytest

REPO = Path(__file__).parent.parent
MSG_ROOTS = [
    REPO / "third_party" / "common_interfaces",
    REPO / "scratch" / "common_interfaces",
]

ROS2_DIR = REPO / "esphome" / "components" / "ros2"

EXPECTED_MSG_FIELDS = {
    "sensor_msgs/msg/JointState.msg": ["header", "name", "position", "velocity", "effort"],
    "trajectory_msgs/msg/JointTrajectory.msg": ["header", "joint_names", "points"],
    "trajectory_msgs/msg/JointTrajectoryPoint.msg": [
        "positions", "velocities", "accelerations", "effort", "time_from_start",
    ],
    "std_msgs/msg/Header.msg": ["stamp", "frame_id"],
    "std_msgs/msg/Bool.msg": ["data"],
    "std_msgs/msg/Float32.msg": ["data"],
    "std_msgs/msg/Int32.msg": ["data"],
    "std_msgs/msg/String.msg": ["data"],
}

# JSON keys the codec must handle, per canonical field names above.
EXPECTED_WIRE_KEYS = [
    "name", "position", "velocity", "effort",
    "joint_names", "points", "positions", "data",
]

# Struct members mirroring canonical fields (bounded MCU projection).
EXPECTED_MEMBERS = [
    "char name[", "float position[", "float velocity[", "float effort[",
    "char joint_names[", "float positions[", "float velocities[",
    "float accelerations[", "stamp_sec", "stamp_nsec", "frame_id",
]


def find_msg_root():
    for root in MSG_ROOTS:
        if root.is_dir():
            return root
    return None


def parse_msg_fields(path: Path):
    fields = []
    for line in path.read_text().splitlines():
        line = line.split("#", 1)[0].strip()
        if not line or "=" in line:
            continue
        parts = line.split()
        if len(parts) >= 2:
            fields.append(parts[1])
    return fields


msg_root = find_msg_root()


@pytest.mark.skipif(msg_root is None, reason="common_interfaces submodule not checked out")
@pytest.mark.parametrize("rel,expected", list(EXPECTED_MSG_FIELDS.items()))
def test_msg_fields_match_canonical(rel, expected):
    assert parse_msg_fields(msg_root / rel) == expected


def _read(name):
    return (ROS2_DIR / name).read_text()


def test_wire_keys_cover_canonical_fields():
    codec = _read("ros2_json.cpp")
    for key in EXPECTED_WIRE_KEYS:
        assert f'root["{key}"]' in codec or f'p0["{key}"]' in codec, key


def test_no_plural_alias_for_joint_state_name():
    codec = _read("ros2_json.cpp")
    assert 'root["names"]' not in codec


def test_struct_members_mirror_canonical_fields():
    header = _read("ros2_types.h")
    for member in EXPECTED_MEMBERS:
        assert member in header, member
    assert re.search(r"(?<!_)names\[", header) is None
