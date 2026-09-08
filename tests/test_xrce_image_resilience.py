"""Image uploader resilience checks (stdlib-only: no esphome install needed).

Guards the 21kB/s fix: single congested/poisoned frames must not force a
full link drop. Asserts granular counters, soft stream reset, bounded
prepare retry, and dump_config reporting via source grep.
"""
from pathlib import Path

REPO = Path(__file__).parent.parent
COMP = REPO / "esphome" / "components" / "xrce_dds"
H = (COMP / "xrce_dds_component.h").read_text()
CPP = (COMP / "xrce_dds_component.cpp").read_text()


def test_image_counters_declared():
    for name in ("img_ok_", "img_drop_link_down_", "img_drop_no_writer_",
                 "img_drop_prepare_", "img_drop_encode_", "img_encode_fails_"):
        assert name in H, f"missing counter {name}"


def test_prepare_retry_and_distinct_log():
    assert "IMG_PREPARE_RETRY_MS" in CPP
    assert "Image prepare congested" in CPP
    assert CPP.count("uxr_prepare_output_stream_fragmented(&this->session_, this->img_stream_") >= 2


def test_soft_reset_replaces_single_drop():
    assert "reset_img_stream_" in H
    assert "reset_img_stream_" in CPP
    assert "uxr_reset_output_reliable_stream" in CPP
    assert "uxr_get_output_reliable_stream" in CPP
    assert "IMG_ENCODE_FAIL_LIMIT" in CPP
    # Fallback path still exists for persistent failures / dead agents.
    assert CPP.count("this->drop_link_()") >= 2


def test_dump_config_reports_images():
    assert "Images ok:" in CPP
    assert "img_drop_prepare_" in CPP
    # Legacy TX line kept for log parsers.
    assert "TX ok:" in CPP


def test_probe_and_advertising_anchors():
    assert "PROBE tx:" in CPP
    assert "Advertising:" in CPP


def test_track1_mtu_raised_to_wifi_size():
    cfg = (COMP / "vendor" / "microxrcedds" / "include" / "uxr" / "client" / "config.h").read_text()
    assert "#define UXR_CONFIG_CUSTOM_TRANSPORT_MTU 1472" in cfg
    vendor_h = (COMP / "xrce_dds_vendor.h").read_text()
    assert "#define UXR_CONFIG_CUSTOM_TRANSPORT_MTU 1472" in vendor_h
    init_py = (COMP / "__init__.py").read_text()
    assert "default=1472" in init_py
    assert "max_packet_length_{1472}" in H


def test_track1_img_stream_small_slots_deep_history():
    assert "XRCE_IMG_HISTORY" in H
    assert "32" in H  # power of two for vendored seq-num arithmetic
    # 45056/32 = 1408 wire bytes per slot: one UDP datagram, no IP fragments.
    assert "45056" in H
    assert "XRCE_IMG_HISTORY" in CPP
    # 19 kB frames (~14 slots) and 40 kB (~29 slots) fit without mid-frame ACKs.
    assert "1408" in H or "1412" in H


def test_track1_socket_stays_lwip_compatible():
    udp_cpp = (COMP / "xrce_dds_transport_udp.cpp").read_text()
    # lwIP rejects SO_SNDBUF/SO_RCVBUF (ENOPROTOOPT) and lacks netinet/ip.h:
    # no setsockopt tuning, non-blocking contract preserved.
    assert "setsockopt" not in udp_cpp
    assert "netinet/ip.h" not in udp_cpp
    assert "MSG_DONTWAIT" in udp_cpp  # non-blocking contract preserved
    assert "EAGAIN" in udp_cpp


def test_track1_examples_disable_wifi_sleep_and_noisy_logs():
    for name in ("stewart_xrce_dds.yaml", "rover_diff_drive.yaml"):
        text = (REPO / "examples" / name).read_text()
        assert "power_save_mode: none" in text, f"{name} missing wifi power_save_mode"
        assert "level: WARN" in text, f"{name} missing logger level"
