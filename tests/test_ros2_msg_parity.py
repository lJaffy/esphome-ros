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
    "sensor_msgs/msg/CompressedImage.msg": ["header", "format", "data"],
    "sensor_msgs/msg/Joy.msg": ["header", "axes", "buttons"],
    "std_msgs/msg/ColorRGBA.msg": ["r", "g", "b", "a"],
    "trajectory_msgs/msg/JointTrajectory.msg": ["header", "joint_names", "points"],
    "trajectory_msgs/msg/JointTrajectoryPoint.msg": [
        "positions", "velocities", "accelerations", "effort", "time_from_start",
    ],
    "std_msgs/msg/Header.msg": ["stamp", "frame_id"],
    "std_msgs/msg/Bool.msg": ["data"],
    "std_msgs/msg/Float32.msg": ["data"],
    "std_msgs/msg/Int32.msg": ["data"],
    "std_msgs/msg/String.msg": ["data"],
    "sensor_msgs/msg/Range.msg": [
        "header", "radiation_type", "field_of_view", "min_range", "max_range", "range", "variance",
    ],
    "sensor_msgs/msg/BatteryState.msg": [
        "header", "voltage", "temperature", "current", "charge", "capacity", "design_capacity",
        "percentage", "power_supply_status", "power_supply_health", "power_supply_technology",
        "present", "cell_voltage", "cell_temperature", "location", "serial_number",
    ],
    "geometry_msgs/msg/Twist.msg": ["linear", "angular"],
    "geometry_msgs/msg/Vector3.msg": ["x", "y", "z"],
    "geometry_msgs/msg/Quaternion.msg": ["x", "y", "z", "w"],
    "geometry_msgs/msg/Point.msg": ["x", "y", "z"],
    "geometry_msgs/msg/Pose.msg": ["position", "orientation"],
    "geometry_msgs/msg/PoseWithCovariance.msg": ["pose", "covariance"],
    "geometry_msgs/msg/TwistWithCovariance.msg": ["twist", "covariance"],
    "geometry_msgs/msg/Transform.msg": ["translation", "rotation"],
    "geometry_msgs/msg/TransformStamped.msg": ["header", "child_frame_id", "transform"],
    "nav_msgs/msg/Odometry.msg": ["header", "child_frame_id", "pose", "twist"],
    "sensor_msgs/msg/Imu.msg": ["header", "orientation", "orientation_covariance",
                                "angular_velocity", "angular_velocity_covariance",
                                "linear_acceleration", "linear_acceleration_covariance"],
    # tf2_msgs lives in ros2/geometry2, not in this submodule: TFMessage
    # (transforms: TransformStamped[]) is covered by members/keys only.
}

# JSON keys the bridge must handle, per canonical field names above.
# Image keys live in the MQTT transport (ros2_mqtt.cpp), the rest in the
# codec (ros2_json.cpp). Nested keys (stamp/sec/nanosec/frame_id) match by
# quoted substring since they appear as header["..."]/stamp["..."].
EXPECTED_WIRE_KEYS = [
    "name", "position", "velocity", "effort",
    "joint_names", "points", "positions", "data", "format",
    "axes", "buttons", "r", "g", "b", "a",
    "header", "stamp", "sec", "nanosec", "frame_id",
    "radiation_type", "field_of_view", "min_range", "max_range", "range", "variance",
    "voltage", "temperature", "current", "charge", "capacity", "design_capacity",
    "percentage", "power_supply_status", "power_supply_health", "power_supply_technology",
    "present", "location",
    "linear", "angular", "x", "y", "z", "w",
    "child_frame_id", "pose", "twist", "covariance",
    "position", "orientation", "transforms", "transform", "translation", "rotation",
    "orientation_covariance", "angular_velocity", "angular_velocity_covariance",
    "linear_acceleration", "linear_acceleration_covariance",
]

# Struct members mirroring canonical fields (bounded MCU projection).
EXPECTED_MEMBERS = [
    "char name[", "float position[", "float velocity[", "float effort[",
    "char joint_names[", "float positions[", "float velocities[",
    "float accelerations[", "stamp_sec", "stamp_nsec", "frame_id",
    "float axes[", "int32_t buttons[",
    "float r{", "float g{", "float b{", "float a{",
    "uint8_t radiation_type", "float field_of_view", "float min_range", "float max_range",
    "float range{", "float variance{",
    "float voltage{", "float temperature{", "float current{", "float charge{", "float capacity{",
    "float design_capacity{", "float percentage{",
    "uint8_t power_supply_status", "uint8_t power_supply_health", "uint8_t power_supply_technology",
    "bool present", "char location[",
    "float linear_x", "float angular_x",
    "float pose_position[", "float pose_orientation[",
    "float twist_linear[", "float twist_angular[",
    "char child_frame_id[", "float translation[", "float rotation[",
    "uint8_t num_transforms",
    "float orientation[", "float orientation_covariance[",
    "float angular_velocity[", "float angular_velocity_covariance[",
    "float linear_acceleration[", "float linear_acceleration_covariance[",
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
    mqtt = (REPO / "esphome" / "components" / "ros2_mqtt" / "ros2_mqtt.cpp").read_text()
    for key in EXPECTED_WIRE_KEYS:
        in_codec = f'"{key}"' in codec
        in_mqtt = f'\\"{key}\\"' in mqtt
        assert in_codec or in_mqtt, key


def test_no_plural_alias_for_joint_state_name():
    codec = _read("ros2_json.cpp")
    assert 'root["names"]' not in codec


def test_struct_members_mirror_canonical_fields():
    header = _read("ros2_types.h")
    for member in EXPECTED_MEMBERS:
        assert member in header, member
    assert re.search(r"(?<!_)names\[", header) is None
