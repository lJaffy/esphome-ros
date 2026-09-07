"""Raw/filtered decoupling + skip-when-stale schema tests.

Run where ESPHome is installed:
    python -m pytest tests/test_ros2_raw_cache.py -v
"""
import importlib.util
import logging
import sys
from pathlib import Path

import pytest

COMP_INIT = Path(__file__).parent.parent / "esphome" / "components" / "ros2" / "__init__.py"

esphome = pytest.importorskip("esphome")


def load_ros2_init():
    spec = importlib.util.spec_from_file_location("ros2_under_test_raw", COMP_INIT)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["ros2_under_test_raw"] = mod
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope="module")
def ros2():
    return load_ros2_init()


def _sensor_pub(ros2, **kw):
    base = {"topic": "/x", "type": "std_msgs/Float32",
            "source": {"sensor": {"id": "s"}}}
    base.update(kw)
    return ros2.PUBLICATION_SCHEMA(base)


def test_raw_defaults_false(ros2):
    cfg = _sensor_pub(ros2)
    assert cfg["raw"] is False


def test_raw_accepted_on_sensor_backed_types(ros2):
    for t, source in (
        ("std_msgs/Float32", {"sensor": {"id": "s"}}),
        ("sensor_msgs/Range", {"sensor": {"id": "s"}}),
        ("sensor_msgs/BatteryState", {"sensor": {"id": "s"}}),
    ):
        cfg = ros2.PUBLICATION_SCHEMA(
            {"topic": "/x", "type": t, "source": source, "raw": True})
        assert cfg["raw"] is True


def test_raw_accepted_on_imu_and_gps(ros2):
    cfg = ros2.PUBLICATION_SCHEMA(
        {"topic": "/imu", "type": "sensor_msgs/Imu",
         "source": {"imu": {"accel_x": {"id": "ax"}, "accel_y": {"id": "ay"},
                            "accel_z": {"id": "az"}, "gyro_x": {"id": "gx"},
                            "gyro_y": {"id": "gy"}, "gyro_z": {"id": "gz"}}},
         "raw": True})
    assert cfg["raw"] is True
    cfg = ros2.PUBLICATION_SCHEMA(
        {"topic": "/fix", "type": "sensor_msgs/NavSatFix",
         "source": {"gps": {"latitude": {"id": "lat"},
                            "longitude": {"id": "lon"}}},
         "raw": True})
    assert cfg["raw"] is True


def test_raw_rejected_off_sensor_types(ros2):
    with pytest.raises(Exception):
        ros2.PUBLICATION_SCHEMA(
            {"topic": "/x", "type": "std_msgs/Bool",
             "source": {"switch": {"id": "s"}}, "raw": True})
    with pytest.raises(Exception):
        ros2.PUBLICATION_SCHEMA(
            {"topic": "/tf", "type": "tf2_msgs/TFMessage",
             "transforms": [{"frame_id": "a", "child_frame_id": "b",
                             "translation": [0.0, 0.0, 0.0],
                             "rotation": [0.0, 0.0, 0.0, 1.0]}],
             "raw": True})


class _StubFullConfig:
    """Minimal fv.full_config stand-in keyed by sensor ID."""

    def __init__(self, sensors):
        self._sensors = sensors

    def get_path_for_id(self, sid):
        # Validated entity refs are core.ID objects, not plain strings
        # (mirrors config.Config.get_path_for_id, which compares str(id)).
        key = str(sid)
        if key not in self._sensors:
            raise KeyError(sid)
        return ["sensor", key, "id"]

    def get_config_for_path(self, path):
        return {"update_interval": self._sensors[path[1]]}


def _run_final_validate(ros2, pubs, sensors, default_interval="1s"):
    from esphome import final_validate as fv
    import esphome.config_validation as cv

    config = {
        "publications": pubs,
        "default_publish_interval": cv.positive_time_period_milliseconds(
            default_interval),
    }
    token = fv.full_config.set(_StubFullConfig(sensors))
    try:
        ros2._final_validate(config)
    finally:
        fv.full_config.reset(token)


def _ms(ros2, value):
    import esphome.config_validation as cv

    return cv.positive_time_period_milliseconds(value)


def test_final_validate_warns_when_publish_faster_than_sensor(ros2, caplog):
    pub = ros2.PUBLICATION_SCHEMA(
        {"topic": "/x", "type": "std_msgs/Float32",
         "source": {"sensor": {"id": "s"}}, "interval": "100ms"})
    with caplog.at_level(logging.WARNING):
        _run_final_validate(ros2, [pub], {"s": _ms(ros2, "1s")})
    assert any("/x" in r.message and "stale" in r.message
               for r in caplog.records)


def test_final_validate_silent_when_sensor_fast_enough(ros2, caplog):
    pub = ros2.PUBLICATION_SCHEMA(
        {"topic": "/x", "type": "std_msgs/Float32",
         "source": {"sensor": {"id": "s"}}, "interval": "1s"})
    with caplog.at_level(logging.WARNING):
        _run_final_validate(ros2, [pub], {"s": _ms(ros2, "50ms")})
    assert not [r for r in caplog.records if "stale" in r.message]


def test_final_validate_uses_default_interval(ros2, caplog):
    pub = ros2.PUBLICATION_SCHEMA(
        {"topic": "/x", "type": "sensor_msgs/Range",
         "source": {"sensor": {"id": "s"}}})
    with caplog.at_level(logging.WARNING):
        _run_final_validate(ros2, [pub], {"s": _ms(ros2, "10s")},
                            default_interval="1s")
    assert any("stale" in r.message for r in caplog.records)


def test_final_validate_skips_non_sensor_types(ros2, caplog):
    pub = ros2.PUBLICATION_SCHEMA(
        {"topic": "/b", "type": "std_msgs/Bool",
         "source": {"switch": {"id": "s"}}})
    with caplog.at_level(logging.WARNING):
        _run_final_validate(ros2, [pub], {})
    assert not [r for r in caplog.records if "stale" in r.message]


def test_final_validate_skips_unknown_sensor_id(ros2, caplog):
    pub = ros2.PUBLICATION_SCHEMA(
        {"topic": "/x", "type": "std_msgs/Float32",
         "source": {"sensor": {"id": "ghost"}}, "interval": "100ms"})
    with caplog.at_level(logging.WARNING):
        _run_final_validate(ros2, [pub], {"other": _ms(ros2, "10s")})
    assert not [r for r in caplog.records if "stale" in r.message]


def test_final_validate_warns_on_slow_imu_axis(ros2, caplog):
    pub = ros2.PUBLICATION_SCHEMA(
        {"topic": "/imu", "type": "sensor_msgs/Imu",
         "source": {"imu": {"accel_x": {"id": "ax"}, "accel_y": {"id": "ay"},
                            "accel_z": {"id": "az"}, "gyro_x": {"id": "gx"},
                            "gyro_y": {"id": "gy"}, "gyro_z": {"id": "gz"}}},
         "interval": "50ms"})
    sensors = {k: _ms(ros2, "10ms") for k in ("ax", "ay", "az", "gx", "gy")}
    sensors["gz"] = _ms(ros2, "1s")
    with caplog.at_level(logging.WARNING):
        _run_final_validate(ros2, [pub], sensors)
    assert any("gz" in r.message and "stale" in r.message
               for r in caplog.records)


def test_final_validate_no_context_is_silent(ros2):
    pub = ros2.PUBLICATION_SCHEMA(
        {"topic": "/x", "type": "std_msgs/Float32",
         "source": {"sensor": {"id": "s"}}, "interval": "100ms"})
    ros2._final_validate({"publications": [pub]})


def test_cpp_cache_hooks_present():
    cpp = (COMP_INIT.parent / "ros2_component.cpp").read_text()
    for needle in (
        "register_single_sensor_cache_",
        "register_imu_cache_",
        "register_navsat_cache_",
        "single_sample_",
        "imu_sample_",
        "navsat_sample_",
        "set_publication_raw",
        "stale_skips",
        "add_on_raw_state_callback",
    ):
        assert needle in cpp, needle
    header = (COMP_INIT.parent / "ros2_component.h").read_text()
    for needle in ("use_raw", "has_raw", "has_filt", "stale_skips",
                   "imu_raw", "navsat_raw_v", "set_publication_raw"):
        assert needle in header, needle
