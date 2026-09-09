"""Tests for tools/dds_capture (stdlib-only: no cyclonedds, no ROS).

Covers the DDS naming mirror (must match xrce_dds_codec.cpp), the
CompressedImage CDR parser, filename logic, and CLI topic parsing.
"""
import importlib.util
import struct
import sys
from pathlib import Path

import pytest

TOOLS = Path(__file__).parent.parent / "tools" / "dds_capture"
REPO = Path(__file__).parent.parent


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope="module")
def dds():
    return _load("dds_types_under_test", TOOLS / "dds_types.py")


@pytest.fixture(scope="module")
def cap(dds):
    sys.path.insert(0, str(TOOLS))  # so capture.py finds dds_types
    try:
        return _load("capture_under_test", TOOLS / "capture.py")
    finally:
        sys.path.remove(str(TOOLS))


def _cdr_string(s):
    raw = s.encode() + b"\x00"
    return struct.pack("<I", len(raw)) + raw + b"\x00" * (-len(raw) % 4)


def _cdr_image(sec=1234, nsec=5678, frame_id="cam", fmt="jpeg",
               data=b"\xff\xd8\xff\xe0test"):
    return (struct.pack("<iI", sec, nsec) + _cdr_string(frame_id)
            + _cdr_string(fmt) + struct.pack("<I", len(data)) + data)


def test_dds_topic_name(dds):
    assert dds.dds_topic_name("/camera/image/compressed") == \
        "rt/camera/image/compressed"
    assert dds.dds_topic_name("/x") == "rt/x"
    with pytest.raises(ValueError):
        dds.dds_topic_name("nope")


def test_type_map_covers_bridge_types(dds):
    # Every type the ros2 bridge can emit must resolve (key check prevents
    # silent discovery mismatches). Mirrors dds_type_suffix() in the codec.
    for ros in ("std_msgs/Bool", "std_msgs/Float32", "std_msgs/Int32",
                "std_msgs/String", "sensor_msgs/JointState",
                "trajectory_msgs/JointTrajectory", "std_msgs/ColorRGBA",
                "sensor_msgs/Joy", "sensor_msgs/Range",
                "sensor_msgs/BatteryState", "geometry_msgs/Twist",
                "nav_msgs/Odometry", "tf2_msgs/TFMessage", "sensor_msgs/Imu",
                "sensor_msgs/NavSatFix", "sensor_msgs/CompressedImage"):
        assert ros in dds.ROS_TO_DDS_TYPE, ros
    assert dds.ROS_TO_DDS_TYPE["sensor_msgs/CompressedImage"] == \
        "sensor_msgs::msg::dds_::CompressedImage_"


def test_parse_compressed_image_roundtrip(dds):
    data = b"\xff\xd8\xff\xe0test"
    msg = dds.parse_compressed_image(_cdr_image(data=data))
    assert msg["sec"] == 1234
    assert msg["nanosec"] == 5678
    assert msg["frame_id"] == "cam"
    assert msg["format"] == "jpeg"
    assert msg["data"] == data
    assert dds.is_jpeg(msg["data"])


def test_parse_compressed_image_truncated(dds):
    with pytest.raises(ValueError):
        dds.parse_compressed_image(b"\x01\x02")
    with pytest.raises(ValueError):
        dds.parse_compressed_image(_cdr_image()[:-3])


def test_is_jpeg_rejects(dds):
    assert not dds.is_jpeg(b"{}")
    assert not dds.is_jpeg(b"")


def test_frame_filename(dds):
    from datetime import datetime, timezone
    stamp = datetime(2026, 9, 8, 0, 41, 36, tzinfo=timezone.utc)
    name = dds.frame_filename("./frames", stamp, 7)
    assert name == "./frames/frame-000007_20260908T004136Z.jpg"


def test_service_names_mirror_codec(dds):
    req, rep = dds.dds_service_names("/toggle_led")
    assert req == "rq/toggle_ledRequest"
    assert rep == "rr/toggle_ledReply"
    req_t, rep_t = dds.ROS_TO_DDS_SERVICE["std_srvs/Trigger"]
    assert req_t == "std_srvs::srv::dds_::Trigger_Request_"
    assert rep_t == "std_srvs::srv::dds_::Trigger_Response_"
    with pytest.raises(ValueError):
        dds.dds_service_names("no-slash")


def test_requester_uses_bin_create():
    # rmw_microxrcedds uses create_requester_bin (explicit topic/type
    # strings), not XML: no FastDDS get_requester_qos_from_xml dialect to
    # mismatch against. The XML builder must be gone, not just unused.
    src = (REPO / "esphome" / "components" / "xrce_dds"
           / "xrce_dds_component.cpp").read_text()
    assert "uxr_buffer_create_requester_bin" in src
    assert "build_requester_xml" not in src
    assert "uxr_buffer_create_requester_xml" not in src


def test_trigger_server_reply_text():
    srv = _load("trigger_server_under_test", TOOLS / "trigger_server.py")
    assert srv.SERVICE_NAME == "/toggle_led"
    assert srv.build_reply_text(1) == "toggle_led ok #1"
    assert srv.build_reply_text(2) == "toggle_led ok #2"


def test_compose_has_trigger_service():
    text = (TOOLS / "docker-compose.yml").read_text()
    assert "trigger:" in text
    assert 'command: ["trigger"]' in text


def test_dockerfile_no_ros_reinstall():
    # Regression: installing ros-jazzy-rclpy/std-srvs on top of the
    # prebuilt image caused FastDDS version skew inside the jazzy stack
    # (`ros2 service call` died with undefined-symbol while the rclpy
    # server still listed fine). The base image already ships both
    # (verified via `dpkg -s`); only python3-pip may be added.
    text = (TOOLS / "Dockerfile").read_text()
    assert "ros-jazzy-rclpy" not in text
    assert "ros-jazzy-std-srvs" not in text
    assert "python3-pip" in text


def test_dockerfile_trigger_typesupport_smoke():
    # The skew above is invisible until a Trigger client dlopens the
    # fastrtps typesupport, so the build must exercise exactly that.
    text = (TOOLS / "Dockerfile").read_text()
    assert "create_client(Trigger" in text


def test_parse_topic_arg_defaults(cap):
    assert cap.parse_topic_arg("/camera/image/compressed") == \
        ("/camera/image/compressed", "CompressedImage")


def test_parse_topic_arg_explicit(cap):
    assert cap.parse_topic_arg("/joint_states_echo=JointState") == \
        ("/joint_states_echo", "JointState")
    assert cap.parse_topic_arg("/x=sensor_msgs/JointState") == \
        ("/x", "JointState")


def test_parse_topic_arg_rejects(cap):
    with pytest.raises(ValueError):
        cap.parse_topic_arg("no-slash")
    with pytest.raises(ValueError):
        cap.parse_topic_arg("/x=Nope")
    with pytest.raises(ValueError):
        cap.parse_topic_arg("/unknown-topic-xyz")


def test_cli_wires_subcommands(cap):
    args = cap.build_parser().parse_args(["save", "--out", "./o"])
    assert args.func is cap.cmd_save
    args = cap.build_parser().parse_args(
        ["record", "--out", "./o", "--topic", "/a=JointState"])
    assert args.func is cap.cmd_record
    assert cap.resolve_topics(args) == [("/a", "JointState")]
