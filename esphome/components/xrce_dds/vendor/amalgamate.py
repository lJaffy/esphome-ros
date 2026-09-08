#!/usr/bin/env python3
"""Amalgamate the vendored XRCE-DDS C sources into two top-level files.

Why this exists: ESPHome's ESP-IDF pipeline only stages files directly in
the component directory into the build (`loader.py::resources`, no
recursion) and only forwards `-D`/`-W` build flags to CMake
(`framework_helpers.py::get_project_compile_flags`, so `-I` is dropped).
A deep `vendor/` tree therefore can neither be compiled nor found. The
amalgamation works around both limits with zero build-system dependence:
two generated files at the component root are auto-copied and
auto-compiled (`*.c` via `GLOB_RECURSE`), and all project headers are
inlined so no `-I` is needed.

Outputs (written next to the component, i.e. `..`):
  xrce_dds_vendor.h  - all headers under vendor/*/include, inlined
                       depth-first (valid declaration order), with the
                       baked config.h files. Included by our C++ code and
                       by xrce_dds_vendor.c.
  xrce_dds_vendor.c  - `#include "xrce_dds_vendor.h"` plus all 30
                       vendored `.c` bodies and the src-internal headers
                       they use, in upstream `SRCS` order. Compiled as C.

Rules (keep in sync with the freshness test):
  - `#include "..."` resolving inside vendor/ is inlined once (dedupe by
    resolved absolute path); markers `/* === BEGIN <rel> === */` /
    `/* === END <rel> === */` bracket each inlined file.
  - `#include <uxr/...>` / `#include <ucdr/...>` resolving inside vendor/
    is treated the same (headers under include/ always land in the .h,
    even when first reached from a .c).
  - Any other `#include` (system headers, platform-guarded transports we
    did not vendor) is kept verbatim; those sit inside false `#ifdef`s
    under our baked config, exactly as upstream.
  - All preprocessor conditionals are kept verbatim; text is only
    relocated, never rewritten, so guarded semantics are unchanged.

Regenerate: `python3 vendor/amalgamate.py` from esphome/components/xrce_dds/
(or `python3 esphome/components/xrce_dds/vendor/amalgamate.py` from root).
The result must be byte-identical to the committed files
(tests/test_xrce_vendor.py checks this).
"""
import re
import sys
from pathlib import Path

VENDOR = Path(__file__).parent
COMP = VENDOR.parent
UCR_INC = VENDOR / "microcdr" / "include"
UXR_INC = VENDOR / "microxrcedds" / "include"

INCLUDE_RE = re.compile(r'^\s*#\s*include\s*"([^"]+)"')
ANGLE_RE = re.compile(r"^\s*#\s*include\s*<([^>]+)>")

# Upstream CMake SRCS order (Micro-XRCE-DDS-Client) + micro-CDR first.
SOURCES = [
    VENDOR / "microcdr" / "src" / "c" / "common.c",
    VENDOR / "microcdr" / "src" / "c" / "types" / "array.c",
    VENDOR / "microcdr" / "src" / "c" / "types" / "basic.c",
    VENDOR / "microcdr" / "src" / "c" / "types" / "sequence.c",
    VENDOR / "microcdr" / "src" / "c" / "types" / "string.c",
    VENDOR / "microxrcedds" / "src" / "c" / "core" / "session" / "stream" / "input_best_effort_stream.c",
    VENDOR / "microxrcedds" / "src" / "c" / "core" / "session" / "stream" / "input_reliable_stream.c",
    VENDOR / "microxrcedds" / "src" / "c" / "core" / "session" / "stream" / "output_best_effort_stream.c",
    VENDOR / "microxrcedds" / "src" / "c" / "core" / "session" / "stream" / "output_reliable_stream.c",
    VENDOR / "microxrcedds" / "src" / "c" / "core" / "session" / "stream" / "stream_storage.c",
    VENDOR / "microxrcedds" / "src" / "c" / "core" / "session" / "stream" / "stream_id.c",
    VENDOR / "microxrcedds" / "src" / "c" / "core" / "session" / "stream" / "seq_num.c",
    VENDOR / "microxrcedds" / "src" / "c" / "core" / "session" / "session.c",
    VENDOR / "microxrcedds" / "src" / "c" / "core" / "session" / "session_info.c",
    VENDOR / "microxrcedds" / "src" / "c" / "core" / "session" / "submessage.c",
    VENDOR / "microxrcedds" / "src" / "c" / "core" / "session" / "object_id.c",
    VENDOR / "microxrcedds" / "src" / "c" / "core" / "serialization" / "xrce_types.c",
    VENDOR / "microxrcedds" / "src" / "c" / "core" / "serialization" / "xrce_header.c",
    VENDOR / "microxrcedds" / "src" / "c" / "core" / "serialization" / "xrce_subheader.c",
    VENDOR / "microxrcedds" / "src" / "c" / "core" / "session" / "common_create_entities.c",
    VENDOR / "microxrcedds" / "src" / "c" / "core" / "session" / "create_entities_ref.c",
    VENDOR / "microxrcedds" / "src" / "c" / "core" / "session" / "create_entities_xml.c",
    VENDOR / "microxrcedds" / "src" / "c" / "core" / "session" / "create_entities_bin.c",
    VENDOR / "microxrcedds" / "src" / "c" / "core" / "session" / "read_access.c",
    VENDOR / "microxrcedds" / "src" / "c" / "core" / "session" / "write_access.c",
    VENDOR / "microxrcedds" / "src" / "c" / "core" / "log" / "log.c",
    VENDOR / "microxrcedds" / "src" / "c" / "profile" / "transport" / "custom" / "custom_transport.c",
    VENDOR / "microxrcedds" / "src" / "c" / "profile" / "transport" / "stream_framing" / "stream_framing_protocol.c",
    VENDOR / "microxrcedds" / "src" / "c" / "util" / "time.c",
    VENDOR / "microxrcedds" / "src" / "c" / "util" / "ping.c",
]


def map_angle(name):
    if name.startswith("uxr/"):
        return UXR_INC / name
    if name.startswith("ucdr/"):
        return UCR_INC / name
    return None


def is_public(path):
    parts = path.relative_to(VENDOR).parts
    return "include" in parts


class Amalgam:
    def __init__(self):
        self.seen = set()
        self.h = []
        self.c = []

    def rel(self, path):
        return path.relative_to(VENDOR).as_posix()

    def emit_header(self, path):
        key = path.resolve()
        if key in self.seen:
            return
        self.seen.add(key)
        self.h.append("/* === BEGIN %s === */\n" % self.rel(path))
        self._run_lines(path, self.h)
        self.h.append("/* === END %s === */\n" % self.rel(path))

    def emit_body(self, path):
        key = path.resolve()
        if key in self.seen:
            return
        self.seen.add(key)
        is_c = path.suffix == ".c"
        if not is_c:
            # src-internal header reached from a body: inline here.
            self.c.append("/* === BEGIN %s === */\n" % self.rel(path))
            self._run_lines(path, self.c)
            self.c.append("/* === END %s === */\n" % self.rel(path))
            return
        self.c.append("/* === BEGIN %s === */\n" % self.rel(path))
        self._run_lines(path, self.c)
        self.c.append("/* === END %s === */\n" % self.rel(path))

    def _run_lines(self, path, buf):
        with open(path, "r", encoding="utf-8") as f:
            for line in f:
                inc = INCLUDE_RE.match(line)
                ang = ANGLE_RE.match(line) if inc is None else None
                if inc is not None:
                    target = (path.parent / inc.group(1)).resolve()
                    if target.is_file() and VENDOR.resolve() in target.parents:
                        if is_public(target):
                            self.emit_header(target)
                        else:
                            self.emit_body(target)
                        continue
                    buf.append(line)
                elif ang is not None:
                    target = map_angle(ang.group(1))
                    if target is not None and target.is_file():
                        self.emit_header(target)
                        continue
                    buf.append(line)
                else:
                    buf.append(line)


def build():
    missing = [str(p) for p in SOURCES if not p.is_file()]
    if missing:
        raise SystemExit("missing vendored sources: %s" % missing)
    am = Amalgam()
    # Pass 1: every public header reachable from the sources, depth-first.
    for src in SOURCES:
        with open(src, "r", encoding="utf-8") as f:
            text = f.read()
        for line in text.splitlines(keepends=True):
            inc = INCLUDE_RE.match(line)
            ang = ANGLE_RE.match(line) if inc is None else None
            target = None
            if inc is not None:
                cand = (src.parent / inc.group(1)).resolve()
                if cand.is_file() and VENDOR.resolve() in cand.parents and is_public(cand):
                    target = cand
            elif ang is not None:
                cand = map_angle(ang.group(1))
                if cand is not None and cand.is_file():
                    target = cand
            if target is not None:
                am.emit_header(target)
    # Pass 2: bodies + src-internal headers in upstream order.
    for src in SOURCES:
        am.emit_body(src)

    banner_h = (
        "/* Amalgamated XRCE-DDS headers. Generated by vendor/amalgamate.py;\n"
        " * do not hand-edit. See vendor/VENDORED.md. */\n"
        "#pragma once\n"
    )
    banner_c = (
        "/* Amalgamated XRCE-DDS sources (Micro-XRCE-DDS-Client v3.0.2 +\n"
        " * micro-CDR v2.0.2, custom-transport + stream-framing only).\n"
        " * Generated by vendor/amalgamate.py; do not hand-edit. */\n"
        '#include "xrce_dds_vendor.h"\n'
    )
    header = banner_h + "".join(am.h)
    if not header.endswith("\n"):
        header += "\n"
    body = banner_c + "".join(am.c)
    if not body.endswith("\n"):
        body += "\n"
    return header, body


def main():
    header, body = build()
    (COMP / "xrce_dds_vendor.h").write_text(header, encoding="utf-8")
    (COMP / "xrce_dds_vendor.c").write_text(body, encoding="utf-8")
    print("wrote xrce_dds_vendor.h (%d bytes) xrce_dds_vendor.c (%d bytes)"
          % (len(header.encode()), len(body.encode())))


if __name__ == "__main__":
    main()
