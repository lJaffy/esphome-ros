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


def test_supported_types_cover_phase1b(ros2):
    for t in ("std_msgs/ColorRGBA", "sensor_msgs/Joy"):
        assert t in ros2.SUPPORTED_TYPES


def _pub(ros2, **kw):
    base = {"topic": "/x", "type": "std_msgs/Float32"}
    base.update(kw)
    return ros2.PUBLICATION_SCHEMA(base)


def test_colorrgba_sub_requires_light(ros2):
    with pytest.raises(Exception):
        ros2.SUBSCRIPTION_SCHEMA(
            {"topic": "/cmd", "type": "std_msgs/ColorRGBA",
             "target": {"servo": {"id": "s"}}}
        )
    cfg = ros2.SUBSCRIPTION_SCHEMA(
        {"topic": "/cmd", "type": "std_msgs/ColorRGBA",
         "target": {"light": {"id": "lamp"}}}
    )
    assert cfg["target"]["light"]["field"] == "rgb"


def test_joy_sub_requires_light(ros2):
    with pytest.raises(Exception):
        ros2.SUBSCRIPTION_SCHEMA(
            {"topic": "/joy", "type": "sensor_msgs/Joy",
             "target": {"switch": {"id": "s"}}}
        )
    cfg = ros2.SUBSCRIPTION_SCHEMA(
        {"topic": "/joy", "type": "sensor_msgs/Joy",
         "target": {"light": {"id": "lamp", "field": "brightness"}}}
    )
    assert cfg["target"]["light"]["field"] == "brightness"


def test_light_field_rejected_values(ros2):
    with pytest.raises(Exception):
        ros2.SUBSCRIPTION_SCHEMA(
            {"topic": "/cmd", "type": "std_msgs/ColorRGBA",
             "target": {"light": {"id": "lamp", "field": "hue"}}}
        )


def test_colorrgba_pub_requires_light_source(ros2):
    with pytest.raises(Exception):
        _pub(ros2, type="std_msgs/ColorRGBA",
             source={"sensor": {"id": "s"}})
    cfg = _pub(ros2, type="std_msgs/ColorRGBA",
               source={"light": {"id": "lamp"}})
    assert str(cfg["source"]["light"]["id"]) == "lamp"


def test_joy_pub_rejected_subscribe_only(ros2):
    with pytest.raises(Exception):
        _pub(ros2, type="sensor_msgs/Joy", source={"light": {"id": "lamp"}})


def test_unmapped_sub_types_rejected(ros2):
    for t in ("std_msgs/Int32", "std_msgs/String"):
        with pytest.raises(Exception):
            ros2.SUBSCRIPTION_SCHEMA(
                {"topic": "/x", "type": t,
                 "target": {"switch": {"id": "s"}}}
            )


def test_unmapped_pub_types_rejected(ros2):
    for t in ("std_msgs/Int32", "std_msgs/String"):
        with pytest.raises(Exception):
            ros2.PUBLICATION_SCHEMA(
                {"topic": "/x", "type": t,
                 "source": {"sensor": {"id": "s"}}}
            )


def test_telemetry_types_publish_only(ros2):
    for t in ("sensor_msgs/Range", "sensor_msgs/BatteryState"):
        with pytest.raises(Exception):
            ros2.SUBSCRIPTION_SCHEMA(
                {"topic": "/x", "type": t,
                 "target": {"switch": {"id": "s"}}}
            )


def test_telemetry_types_accept_sensor_source(ros2):
    for t in ("sensor_msgs/Range", "sensor_msgs/BatteryState"):
        cfg = ros2.PUBLICATION_SCHEMA(
            {"topic": "/x", "type": t,
             "source": {"sensor": {"id": "s"}}}
        )
        assert cfg["type"] == t


def test_telemetry_types_reject_non_sensor_source(ros2):
    with pytest.raises(Exception):
        ros2.PUBLICATION_SCHEMA(
            {"topic": "/x", "type": "sensor_msgs/Range",
             "source": {"switch": {"id": "s"}}}
        )


def test_supported_types_cover_telemetry(ros2):
    for t in ("sensor_msgs/Range", "sensor_msgs/BatteryState"):
        assert t in ros2.SUPPORTED_TYPES


def test_frame_id_rejected_without_header(ros2):
    with pytest.raises(Exception):
        _pub(ros2, type="std_msgs/Float32",
             source={"sensor": {"id": "s"}}, frame_id="base_link")


def test_frame_id_accepted_with_header(ros2):
    cfg = _pub(ros2, type="sensor_msgs/Range",
               source={"sensor": {"id": "s"}}, frame_id="sonar")
    assert cfg["frame_id"] == "sonar"


def test_frame_id_charset_rejected(ros2):
    with pytest.raises(Exception):
        _pub(ros2, type="sensor_msgs/Range",
             source={"sensor": {"id": "s"}}, frame_id='a"b')


def test_range_params_rejected_off_type(ros2):
    with pytest.raises(Exception):
        _pub(ros2, type="std_msgs/Float32",
             source={"sensor": {"id": "s"}}, min_range=0.1)


def test_battery_params_rejected_off_type(ros2):
    with pytest.raises(Exception):
        _pub(ros2, type="std_msgs/Float32",
             source={"sensor": {"id": "s"}}, min_voltage=3.0)


def test_range_bounds_validated(ros2):
    with pytest.raises(Exception):
        ros2.PUBLICATION_SCHEMA(
            {"topic": "/x", "type": "sensor_msgs/Range",
             "source": {"sensor": {"id": "s"}},
             "min_range": 5.0, "max_range": 1.0}
        )


def test_battery_bounds_validated(ros2):
    with pytest.raises(Exception):
        ros2.PUBLICATION_SCHEMA(
            {"topic": "/x", "type": "sensor_msgs/BatteryState",
             "source": {"sensor": {"id": "s"}},
             "min_voltage": 4.2, "max_voltage": 3.0}
        )


def test_qos_accepted_on_sub_and_pub(ros2):
    cfg = ros2.SUBSCRIPTION_SCHEMA(
        {"topic": "/cmd", "type": "std_msgs/ColorRGBA",
         "target": {"light": {"id": "lamp"}}, "qos": "best_effort"}
    )
    assert cfg["qos"] == "best_effort"
    cfg = _pub(ros2, type="sensor_msgs/Range",
               source={"sensor": {"id": "s"}}, qos="best_effort")
    assert cfg["qos"] == "best_effort"


def test_qos_rejects_unknown_level(ros2):
    with pytest.raises(Exception):
        _pub(ros2, type="std_msgs/Float32",
             source={"sensor": {"id": "s"}}, qos="sometimes")


def _twist_sub(ros2, **kw):
    base = {"topic": "/cmd_vel", "type": "geometry_msgs/Twist",
            "target": {"diff_drive": {"left": {"id": "l"}, "right": {"id": "r"},
                                      "wheel_separation": 0.2}}}
    base.update(kw)
    return ros2.SUBSCRIPTION_SCHEMA(base)


def test_twist_requires_diff_drive(ros2):
    with pytest.raises(Exception):
        ros2.SUBSCRIPTION_SCHEMA(
            {"topic": "/cmd_vel", "type": "geometry_msgs/Twist",
             "target": {"servo": {"id": "s"}}}
        )


def test_diff_drive_requires_twist(ros2):
    with pytest.raises(Exception):
        ros2.SUBSCRIPTION_SCHEMA(
            {"topic": "/x", "type": "std_msgs/Float32",
             "target": {"diff_drive": {"left": {"id": "l"}, "right": {"id": "r"},
                                       "wheel_separation": 0.2}}}
        )


def test_valid_diff_drive(ros2):
    cfg = _twist_sub(ros2)
    assert cfg["target"]["diff_drive"]["wheel_separation"] == 0.2


def test_odom_needs_odom_source(ros2):
    with pytest.raises(Exception):
        _pub(ros2, type="nav_msgs/Odometry",
             source={"sensor": {"id": "s"}})


def test_valid_odom_source(ros2):
    cfg = _pub(ros2, type="nav_msgs/Odometry",
               source={"odom": {"wheel_separation": 0.2}},
               frame_id="odom", child_frame_id="base_link",
               tf_topic="/tf")
    assert cfg["child_frame_id"] == "base_link"
    assert cfg["tf_topic"] == "/tf"


def test_tf_needs_transforms(ros2):
    with pytest.raises(Exception):
        ros2.PUBLICATION_SCHEMA(
            {"topic": "/tf", "type": "tf2_msgs/TFMessage"}
        )


def test_tf_rejects_source(ros2):
    with pytest.raises(Exception):
        ros2.PUBLICATION_SCHEMA(
            {"topic": "/tf", "type": "tf2_msgs/TFMessage",
             "source": {"sensor": {"id": "s"}},
             "transforms": [{"frame_id": "base_link", "child_frame_id": "laser",
                             "translation": [0.1, 0.0, 0.2],
                             "rotation": [0.0, 0.0, 0.0, 1.0]}]}
        )


def _tf_pub(ros2, **kw):
    base = {"topic": "/tf", "type": "tf2_msgs/TFMessage",
            "transforms": [{"frame_id": "base_link", "child_frame_id": "laser",
                            "translation": [0.1, 0.0, 0.2],
                            "rotation": [0.0, 0.0, 0.0, 1.0]}]}
    base.update(kw)
    return ros2.PUBLICATION_SCHEMA(base)


def test_valid_tf(ros2):
    cfg = _tf_pub(ros2)
    assert len(cfg["transforms"]) == 1


def test_transforms_rejected_off_type(ros2):
    with pytest.raises(Exception):
        _pub(ros2, type="std_msgs/Float32",
             source={"sensor": {"id": "s"}},
             transforms=[{"frame_id": "a", "child_frame_id": "b",
                          "translation": [0.0, 0.0, 0.0],
                          "rotation": [0.0, 0.0, 0.0, 1.0]}])


def test_tf_topic_rejected_off_type(ros2):
    with pytest.raises(Exception):
        _pub(ros2, type="std_msgs/Float32",
             source={"sensor": {"id": "s"}}, tf_topic="/tf")


def test_child_frame_id_rejected_off_type(ros2):
    with pytest.raises(Exception):
        _pub(ros2, type="sensor_msgs/Range",
             source={"sensor": {"id": "s"}}, child_frame_id="base_link")


def test_supported_types_cover_motion(ros2):
    for t in ("geometry_msgs/Twist", "nav_msgs/Odometry", "tf2_msgs/TFMessage"):
        assert t in ros2.SUPPORTED_TYPES


def _imu_pub(ros2, **kw):
    base = {"topic": "/imu", "type": "sensor_msgs/Imu",
            "source": {"imu": {"accel_x": {"id": "ax"}, "accel_y": {"id": "ay"},
                               "accel_z": {"id": "az"}, "gyro_x": {"id": "gx"},
                               "gyro_y": {"id": "gy"}, "gyro_z": {"id": "gz"}}}}
    base.update(kw)
    return ros2.PUBLICATION_SCHEMA(base)


def test_imu_publish_only(ros2):
    with pytest.raises(Exception):
        ros2.SUBSCRIPTION_SCHEMA(
            {"topic": "/imu", "type": "sensor_msgs/Imu",
             "target": {"switch": {"id": "s"}}}
        )


def test_imu_needs_imu_source(ros2):
    with pytest.raises(Exception):
        _pub(ros2, type="sensor_msgs/Imu",
             source={"sensor": {"id": "s"}})


def test_valid_imu_source(ros2):
    cfg = _imu_pub(ros2, frame_id="imu_link")
    assert cfg["frame_id"] == "imu_link"


def test_imu_orientation_all_or_none(ros2):
    with pytest.raises(Exception):
        _imu_pub(ros2, source={"imu": {
            "accel_x": {"id": "ax"}, "accel_y": {"id": "ay"},
            "accel_z": {"id": "az"}, "gyro_x": {"id": "gx"},
            "gyro_y": {"id": "gy"}, "gyro_z": {"id": "gz"},
            "orientation_x": {"id": "ox"}}})
    cfg = ros2.PUBLICATION_SCHEMA(
        {"topic": "/imu", "type": "sensor_msgs/Imu",
         "source": {"imu": {
             "accel_x": {"id": "ax"}, "accel_y": {"id": "ay"},
             "accel_z": {"id": "az"}, "gyro_x": {"id": "gx"},
             "gyro_y": {"id": "gy"}, "gyro_z": {"id": "gz"},
             "orientation_x": {"id": "ox"}, "orientation_y": {"id": "oy"},
             "orientation_z": {"id": "oz"}, "orientation_w": {"id": "ow"}}}}
    )
    assert cfg["type"] == "sensor_msgs/Imu"


def test_imu_rejects_odom_source(ros2):
    with pytest.raises(Exception):
        _pub(ros2, type="sensor_msgs/Imu",
             source={"odom": {"wheel_separation": 0.2}})


def test_supported_types_cover_imu(ros2):
    assert "sensor_msgs/Imu" in ros2.SUPPORTED_TYPES


def test_auto_load_full_without_config(ros2):
    libs = ros2._auto_load()
    for lib in ("json", "binary_sensor", "sensor", "switch", "servo", "camera", "light"):
        assert lib in libs


def test_auto_load_servo_only(ros2):
    libs = ros2._auto_load(
        {"subscriptions": [{"target": {"servo": {}}}],
         "publications": [{"sources": [{}]}]})
    assert "servo" in libs
    assert "camera" not in libs
    assert "light" not in libs
    assert "json" in libs


def test_auto_load_camera_only(ros2):
    libs = ros2._auto_load(
        {"subscriptions": [],
         "publications": [{"source": {"camera": {}}}]})
    assert "camera" in libs
    assert "servo" not in libs
    assert "light" not in libs


def test_auto_load_light_only(ros2):
    libs = ros2._auto_load(
        {"subscriptions": [{"target": {"light": {}}}],
         "publications": [{"source": {"light": {}}}]})
    assert "light" in libs
    assert "servo" not in libs
    assert "camera" not in libs


def test_auto_load_status_sensor_pulls_binary_sensor(ros2):
    libs = ros2._auto_load(
        {"subscriptions": [], "publications": [],
         "status_sensor": {"id": "s"}})
    assert "binary_sensor" in libs


def test_entity_guards_cover_all_dispatch(ros2):
    import re
    cpp = (COMP_INIT.parent / "ros2_component.cpp").read_text()
    stack = []
    guarded = {}
    pos = 0
    use_re = (r"Camera::instance(?=\()|[A-Za-z_][\w:>-]*->[A-Za-z_]\w*\(|[A-Za-z_]\w*(?=\()")
    for m in re.finditer(r"#(ifdef|ifndef|else|endif)\s*(\w+)?", cpp):
        for mm in re.finditer(use_re, cpp[pos:m.start()]):
            guarded.setdefault(mm.group(0), set()).update(stack)
        tag, name = m.group(1), m.group(2)
        if tag in ("ifdef", "ifndef"):
            stack.append(name)
        elif tag == "endif":
            stack.pop()
        pos = m.end()
    for mm in re.finditer(use_re, cpp[pos:]):
        guarded.setdefault(mm.group(0), set()).update(stack)
    for define, needles in (
        ("USE_SERVO", ["servo->write("]),
        ("USE_LIGHT", ["light->turn_on(", "light->make_call("]),
        ("USE_CAMERA", ["Camera::instance"]),
    ):
        for needle in needles:
            assert needle in guarded, needle
            assert define in guarded[needle], f"{needle} outside #{define}"


def test_optional_entity_includes_guarded(ros2):
    import re
    header = (COMP_INIT.parent / "ros2_component.h").read_text()
    stack = []
    for line in header.splitlines():
        stripped = line.strip()
        m = re.match(r"#(ifdef|ifndef|if|else|elif|endif)\b\s*(\w+)?", stripped)
        if m:
            tag, name = m.group(1), m.group(2)
            if tag in ("ifdef", "ifndef", "if"):
                stack.append(name)
            elif tag == "endif" and stack:
                stack.pop()
            continue
        m = re.match(r'#include "esphome/components/(\w+)/', stripped)
        if m:
            assert stack and (stack[-1] or "").startswith("USE_"), \
                f"{m.group(1)} include outside USE_* guard"
