"""Vendored XRCE-DDS tree checks (stdlib-only: no esphome install needed).

Guards the add-on build: external_components never inits submodules, and
ESPHome's ESP-IDF writer neither copies subdirectories into the build nor
forwards -I flags to CMake. The C sources therefore ship as the committed
amalgamation (xrce_dds_vendor.{h,c}, built by vendor/amalgamate.py), which
needs no extra include dirs and compiles from the component root.
"""
import importlib.util
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).parent.parent
COMP = REPO / "esphome" / "components" / "xrce_dds"
VENDOR = COMP / "vendor"
UXR_INC = VENDOR / "microxrcedds" / "include" / "uxr" / "client"
UCDR_INC = VENDOR / "microcdr" / "include" / "ucdr"

EXPECTED_C = [
    # micro-CDR (5)
    "microcdr/src/c/common.c",
    "microcdr/src/c/types/array.c",
    "microcdr/src/c/types/basic.c",
    "microcdr/src/c/types/sequence.c",
    "microcdr/src/c/types/string.c",
    # core session/streams/serialization (17)
    "microxrcedds/src/c/core/session/stream/input_best_effort_stream.c",
    "microxrcedds/src/c/core/session/stream/input_reliable_stream.c",
    "microxrcedds/src/c/core/session/stream/output_best_effort_stream.c",
    "microxrcedds/src/c/core/session/stream/output_reliable_stream.c",
    "microxrcedds/src/c/core/session/stream/stream_storage.c",
    "microxrcedds/src/c/core/session/stream/stream_id.c",
    "microxrcedds/src/c/core/session/stream/seq_num.c",
    "microxrcedds/src/c/core/session/session.c",
    "microxrcedds/src/c/core/session/session_info.c",
    "microxrcedds/src/c/core/session/submessage.c",
    "microxrcedds/src/c/core/session/object_id.c",
    "microxrcedds/src/c/core/serialization/xrce_types.c",
    "microxrcedds/src/c/core/serialization/xrce_header.c",
    "microxrcedds/src/c/core/serialization/xrce_subheader.c",
    "microxrcedds/src/c/core/session/common_create_entities.c",
    "microxrcedds/src/c/core/session/create_entities_ref.c",
    "microxrcedds/src/c/core/session/create_entities_xml.c",
    # (bin + access + custom + framing + utils below)
    "microxrcedds/src/c/core/session/create_entities_bin.c",
    "microxrcedds/src/c/core/session/read_access.c",
    "microxrcedds/src/c/core/session/write_access.c",
    "microxrcedds/src/c/profile/transport/custom/custom_transport.c",
    "microxrcedds/src/c/profile/transport/stream_framing/stream_framing_protocol.c",
    "microxrcedds/src/c/util/time.c",
    "microxrcedds/src/c/util/ping.c",
    # log.c ships unconditionally: session.c always includes
    # log_internal.h, upstream only links log.c with verbose flags.
    "microxrcedds/src/c/core/log/log.c",
]

# Upstream files that must NOT be vendored (glob-compile hazards:
# duplicate mains, termios/pthread platform sources, unneeded profiles).
BANNED_SUFFIXES = (
    "udp_transport_posix.c",
    "udp_transport_posix_nopoll.c",
    "udp_transport_windows.c",
    "tcp_transport_posix.c",
    "serial_transport_posix.c",
    "serial_transport.c",
    "can_transport_posix.c",
    "discovery.c",
    "matching.c",
    "multithread.c",
    "shared_memory.c",
    "ip_posix.c",
    "ip_windows.c",
)


def test_vendor_tree_present_and_submodules_gone():
    assert VENDOR.is_dir(), "vendor/ tree missing"
    assert not (COMP / "third_party" / "Micro-XRCE-DDS-Client").exists()
    assert not (COMP / "third_party" / "micro-CDR").exists()
    gitmodules = (REPO / ".gitmodules").read_text()
    assert "Micro-XRCE-DDS-Client" not in gitmodules
    assert "micro-CDR" not in gitmodules


def test_expected_sources_present():
    missing = [p for p in EXPECTED_C if not (VENDOR / p).is_file()]
    assert not missing, f"missing vendored sources: {missing}"


def test_no_banned_platform_sources():
    bad = [str(p.relative_to(VENDOR)) for p in VENDOR.rglob("*.c")
           if p.name in BANNED_SUFFIXES]
    assert not bad, f"platform/profile sources must not be vendored: {bad}"


def test_no_int_main_in_vendor():
    bad = []
    for p in list(VENDOR.rglob("*.c")) + list(VENDOR.rglob("*.h")):
        text = p.read_text(errors="ignore")
        if "int main" in text:
            bad.append(str(p.relative_to(VENDOR)))
    assert not bad, f"example/test mains would break the ESPHome link: {bad}"


def test_baked_config_headers():
    uxr_cfg = (UXR_INC / "config.h").read_text()
    for want in (
        "UCLIENT_PROFILE_CUSTOM_TRANSPORT",
        "UCLIENT_PROFILE_STREAM_FRAMING",
        "UXR_CONFIG_CUSTOM_TRANSPORT_MTU",
        "UCLIENT_TWEAK_XRCE_WRITE_LIMIT",
        '#define UXR_CLIENT_VERSION_STR "3.0.2"',
    ):
        assert want in uxr_cfg, f"uxr config.h missing: {want}"
    for banned in (
        "UCLIENT_PROFILE_UDP",
        "UCLIENT_PROFILE_TCP",
        "UCLIENT_PROFILE_SERIAL",
        "UCLIENT_PROFILE_DISCOVERY",
        "UCLIENT_PROFILE_MULTITHREAD",
        "UCLIENT_PROFILE_SHARED_MEMORY",
    ):
        assert banned not in uxr_cfg, f"uxr config.h must not enable {banned}"
    assert not (UXR_INC / "config.h.in").exists(), "unbaked config.h.in shipped"
    ucdr_cfg = (UCDR_INC / "config.h").read_text()
    assert "UCDR_LITTLE_ENDIANNESS" in ucdr_cfg
    assert '#define MICROCDR_VERSION_STR "2.0.2"' in ucdr_cfg
    assert not (UCDR_INC / "config.h.in").exists()


def test_top_level_headers_and_licenses():
    for h in ("client.h", "transport.h", "defines.h", "visibility.h"):
        assert (UXR_INC / h).is_file(), f"missing {h}"
    assert (UXR_INC / "profile" / "transport" / "custom" / "custom_transport.h").is_file()
    assert (UCDR_INC / "microcdr.h").is_file()
    assert (VENDOR / "microxrcedds" / "LICENSE").is_file()
    assert (VENDOR / "microcdr" / "LICENSE").is_file()
    assert (VENDOR / "VENDORED.md").is_file()


def test_init_points_at_vendor():
    # No -I flags: the IDF writer drops them (only -D/-W are forwarded),
    # so the amalgamation must be include-dir free.
    text = (COMP / "__init__.py").read_text()
    assert "add_build_flag" not in text
    assert "microxrcedds" not in text and "microcdr" not in text
    assert "Micro-XRCE-DDS-Client" not in text
    assert "micro-CDR" not in text


def test_our_code_uses_amalgam():
    for name in ("xrce_dds_codec.h", "xrce_dds_component.h"):
        text = (COMP / name).read_text()
        assert '#include "xrce_dds_vendor.h"' in text, name
        assert "<ucdr/" not in text and "<uxr/" not in text, name


def _load_amalgamator():
    path = VENDOR / "amalgamate.py"
    spec = importlib.util.spec_from_file_location("amalgamate_under_test", path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["amalgamate_under_test"] = mod
    spec.loader.exec_module(mod)
    return mod


def test_amalgamation_fresh():
    # The committed xrce_dds_vendor.{h,c} must be byte-identical to what
    # vendor/amalgamate.py produces from vendor/ (no hand-edits, no drift
    # from the perl-generated original).
    am = _load_amalgamator()
    header, body = am.build()
    assert (COMP / "xrce_dds_vendor.h").read_text(encoding="utf-8") == header
    assert (COMP / "xrce_dds_vendor.c").read_text(encoding="utf-8") == body


def test_amalgam_covers_all_sources():
    body = (COMP / "xrce_dds_vendor.c").read_text(encoding="utf-8")
    for rel in EXPECTED_C:
        assert ("/* === BEGIN %s" % rel) in body, rel
    # Every vendored public header is inlined exactly once into the .h.
    header = (COMP / "xrce_dds_vendor.h").read_text(encoding="utf-8")
    assert header.startswith("/* Amalgamated XRCE-DDS headers.")
    assert body.startswith("/* Amalgamated XRCE-DDS sources.")
    assert '#include "xrce_dds_vendor.h"' in body
    for h in ("microcdr/include/ucdr/microcdr.h",
              "microxrcedds/include/uxr/client/client.h",
              "microxrcedds/include/uxr/client/config.h",
              "microcdr/include/ucdr/config.h"):
        assert header.count("/* === BEGIN %s === */" % h) == 1, h
    # No project include may survive unresolved: only the umbrella and
    # system headers remain.
    for line in body.splitlines():
        s = line.strip()
        if s.startswith("#include"):
            assert s == '#include "xrce_dds_vendor.h"' or s.startswith("#include <"), s
