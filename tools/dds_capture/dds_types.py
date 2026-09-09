"""Shared DDS naming + CDR helpers for the S3 capture utility.

Stdlib-only: importable without cyclonedds installed (unit-tested).
Mirrors esphome/components/xrce_dds/xrce_dds_codec.cpp — keep in sync.
"""

import struct
from datetime import datetime, timezone

# ROS topic -> DDS topic: "/x" becomes "rt/x" (dds_topic_name in the codec).
# The leading slash is stripped: "/camera/image/compressed" -> "rt/camera/image/compressed".
DDS_TOPIC_PREFIX = "rt/"


def dds_topic_name(ros_topic):
    """Mirror of dds_topic_name(): '/x' -> 'rt/x'."""
    if not ros_topic or not ros_topic.startswith("/"):
        raise ValueError("ROS topic must start with '/': %r" % (ros_topic,))
    return DDS_TOPIC_PREFIX + ros_topic[1:]


# ROS type -> DDS type suffix. Mirror of dds_type_suffix() in
# xrce_dds_codec.cpp. The Python subscriber must declare the same type
# name or discovery silently never matches.
ROS_TO_DDS_TYPE = {
    "std_msgs/Bool": "std_msgs::msg::dds_::Bool_",
    "std_msgs/Float32": "std_msgs::msg::dds_::Float32_",
    "std_msgs/Int32": "std_msgs::msg::dds_::Int32_",
    "std_msgs/String": "std_msgs::msg::dds_::String_",
    "sensor_msgs/JointState": "sensor_msgs::msg::dds_::JointState_",
    "trajectory_msgs/JointTrajectory": "trajectory_msgs::msg::dds_::JointTrajectory_",
    "std_msgs/ColorRGBA": "std_msgs::msg::dds_::ColorRGBA_",
    "sensor_msgs/Joy": "sensor_msgs::msg::dds_::Joy_",
    "sensor_msgs/Range": "sensor_msgs::msg::dds_::Range_",
    "sensor_msgs/BatteryState": "sensor_msgs::msg::dds_::BatteryState_",
    "geometry_msgs/Twist": "geometry_msgs::msg::dds_::Twist_",
    "nav_msgs/Odometry": "nav_msgs::msg::dds_::Odometry_",
    "tf2_msgs/TFMessage": "tf2_msgs::msg::dds_::TFMessage_",
    "sensor_msgs/Imu": "sensor_msgs::msg::dds_::Imu_",
    "sensor_msgs/NavSatFix": "sensor_msgs::msg::dds_::NavSatFix_",
    "sensor_msgs/CompressedImage": "sensor_msgs::msg::dds_::CompressedImage_",
}

ROS_TO_DDS_SERVICE = {
    "std_srvs/Trigger": (
        "std_srvs::srv::dds_::Trigger_Request_",
        "std_srvs::srv::dds_::Trigger_Response_",
    ),
}


def dds_service_names(ros_service):
    """Mirror of dds_service_request_names(): '/x' -> ('rq/xRequest', 'rr/xReply').

    Suffixes come from rmw_microxrcedds generate_service_topics(); without
    them the agent endpoints never match a ROS 2 server (silent timeout).
    """
    if not ros_service or not ros_service.startswith("/"):
        raise ValueError("ROS service must start with '/': %r" % (ros_service,))
    return "rq" + ros_service + "Request", "rr" + ros_service + "Reply"


# Topics the S3 publishes by default (override with --topic on the CLI).
DEFAULT_TOPICS = {
    "/camera/image/compressed": "sensor_msgs/CompressedImage",
}

JPEG_MAGIC = b"\xff\xd8"


def _read_cdr_string(buf, off, endian):
    """Read a CDR string at offset; return (value, next_offset)."""
    (n,) = struct.unpack_from(endian + "I", buf, off)
    off += 4
    raw = bytes(buf[off:off + n])
    if len(raw) < n:
        raise ValueError("truncated CDR string")
    value = raw[:-1].decode("utf-8", "replace") if n else ""
    off += n + (-n % 4)
    return value, off


def parse_compressed_image(buf):
    """Parse an S3 CompressedImage CDR sample (little-endian, ESP32-S3).

    Layout (no encapsulation header — micro-CDR writes members directly):
      int32 stamp.sec, uint32 stamp.nanosec, string frame_id,
      string format, sequence<octet> data (uint32 length + bytes).
    Returns dict(sec, nanosec, frame_id, format, data). Raises ValueError.
    """
    buf = bytes(buf)
    endian = "<"
    if len(buf) < 8:
        raise ValueError("sample too short: %d bytes" % len(buf))
    sec, nsec = struct.unpack_from(endian + "iI", buf, 0)
    frame_id, off = _read_cdr_string(buf, 8, endian)
    fmt, off = _read_cdr_string(buf, off, endian)
    if off + 4 > len(buf):
        raise ValueError("truncated data length")
    (dlen,) = struct.unpack_from(endian + "I", buf, off)
    off += 4
    data = bytes(buf[off:off + dlen])
    if len(data) < dlen:
        raise ValueError("truncated data: want %d, have %d" % (dlen, len(buf) - off))
    return {"sec": sec, "nanosec": nsec, "frame_id": frame_id,
            "format": fmt, "data": data}


def is_jpeg(data):
    return bytes(data)[:2] == JPEG_MAGIC


def frame_filename(out_dir, stamp, index, ext="jpg"):
    """frame-000123_20260908T004136Z.jpg style names (receive-time based)."""
    ts = stamp.strftime("%Y%m%dT%H%M%SZ")
    return "%s/frame-%06d_%s.%s" % (out_dir.rstrip("/"), index, ts, ext)


def utcnow():
    return datetime.now(timezone.utc)
