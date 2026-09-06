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
