"""Phase 3 schema-validation tests for ros2 service clients.

Run where ESPHome is installed:
    python -m pytest tests/test_ros2_services.py
"""
import importlib.util
import sys
from pathlib import Path

import pytest

COMP_INIT = Path(__file__).parent.parent / "esphome" / "components" / "ros2" / "__init__.py"

esphome = pytest.importorskip("esphome")


def load_ros2_init():
    spec = importlib.util.spec_from_file_location("ros2_svc_under_test", COMP_INIT)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["ros2_svc_under_test"] = mod
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope="module")
def ros2():
    return load_ros2_init()


def _svc(ros2, **kw):
    base = {"service": "/toggle_led", "type": "std_srvs/Trigger",
            "trigger": {"switch": {"id": "trig"}}}
    base.update(kw)
    return ros2.SERVICE_SCHEMA(base)


def test_valid_trigger_client(ros2):
    cfg = _svc(ros2)
    assert cfg["service"] == "/toggle_led"
    assert cfg["timeout"] == 5000


def test_unknown_service_type_rejected(ros2):
    with pytest.raises(Exception):
        ros2.SERVICE_SCHEMA(
            {"service": "/x", "type": "std_srvs/SetBool",
             "trigger": {"switch": {"id": "t"}}}
        )


def test_missing_trigger_rejected(ros2):
    with pytest.raises(Exception):
        ros2.SERVICE_SCHEMA(
            {"service": "/x", "type": "std_srvs/Trigger"}
        )


def test_trigger_needs_switch(ros2):
    with pytest.raises(Exception):
        _svc(ros2, trigger={})


def test_qos_rejected_on_service(ros2):
    with pytest.raises(Exception):
        _svc(ros2, qos="reliable")


def test_frame_id_rejected_on_service(ros2):
    with pytest.raises(Exception):
        _svc(ros2, frame_id="base_link")


def test_supported_service_types_cover_trigger(ros2):
    assert "std_srvs/Trigger" in ros2.SUPPORTED_SERVICE_TYPES


def test_auto_load_service_switch(ros2):
    libs = ros2._auto_load(
        {"subscriptions": [], "publications": [],
         "services": [{"trigger": {"switch": {}}}]})
    assert "switch" in libs
    assert "json" in libs


def test_trigger_srv_parity():
    root = Path(__file__).parent.parent / "third_party" / "common_interfaces"
    srv = root / "std_srvs" / "srv" / "Trigger.srv"
    if not srv.is_file():
        pytest.skip("common_interfaces submodule not checked out")
    text = srv.read_text()
    assert "bool success" in text
    assert "string message" in text


def test_trigger_struct_members():
    header = (Path(__file__).parent.parent / "esphome" / "components" / "ros2"
              / "ros2_types.h").read_text()
    assert "TriggerResMsg" in header
    assert "bool success" in header
    assert "char message[" in header
