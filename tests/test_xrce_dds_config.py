"""Schema-validation tests for the xrce_dds component.

Run where ESPHome is installed (it provides esphome.config_validation):
    python -m pytest tests/test_xrce_dds_config.py
"""
import importlib.util
import sys
from pathlib import Path

import pytest

COMP_INIT = Path(__file__).parent.parent / "esphome" / "components" / "xrce_dds" / "__init__.py"

esphome = pytest.importorskip("esphome")


def load_xrce_init():
    spec = importlib.util.spec_from_file_location("xrce_dds_under_test", COMP_INIT)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["xrce_dds_under_test"] = mod
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope="module")
def xrce():
    return load_xrce_init()


def _transport(xrce, **kw):
    base = {"type": "udp"}
    base.update(kw)
    return xrce.TRANSPORT_SCHEMA(base)


def test_udp_needs_no_ids(xrce):
    cfg = _transport(xrce, type="udp")
    assert cfg["type"] == "udp"


def test_serial_requires_uart_id(xrce):
    with pytest.raises(Exception):
        _transport(xrce, type="serial")


def test_unknown_transport_rejected(xrce):
    with pytest.raises(Exception):
        _transport(xrce, type="tcp")


def test_config_defaults(xrce):
    cfg = xrce.CONFIG_SCHEMA(
        {
            "agent_address": "192.168.1.10",
            "transport": {"type": "udp"},
        }
    )
    assert cfg["agent_port"] == 8888
    assert cfg["domain_id"] == 0
    assert cfg["max_topics"] == 16
    assert cfg["max_datawriters"] == 8
    assert cfg["max_datareaders"] == 8


def test_dependencies_include_network(xrce):
    assert "ros2" in xrce.DEPENDENCIES
    assert "network" in xrce.DEPENDENCIES


def _config(xrce, **kw):
    base = {
        "agent_address": "192.168.1.10",
        "transport": {"type": "udp"},
    }
    base.update(kw)
    return xrce.CONFIG_SCHEMA(base)


def test_max_topics_clamped_to_compile_cap(xrce):
    with pytest.raises(Exception):
        _config(xrce, max_topics=17)


def test_max_endpoints_clamped_to_compile_cap(xrce):
    with pytest.raises(Exception):
        _config(xrce, max_datawriters=9)
    with pytest.raises(Exception):
        _config(xrce, max_datareaders=9)


def test_max_bounds_accepted(xrce):
    cfg = _config(xrce, max_topics=16, max_datawriters=8, max_datareaders=8)
    assert cfg["max_topics"] == 16
    assert cfg["max_datawriters"] == 8
    assert cfg["max_datareaders"] == 8
