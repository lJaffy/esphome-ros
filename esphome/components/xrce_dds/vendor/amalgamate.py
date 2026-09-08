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
  - `#include "..."` resolving inside vendor/ is inlined (dedupe by
    resolved absolute path); markers `/* === BEGIN <rel> === */` /
    `/* === END <rel> === */` bracket each inlined file.
  - `#include <uxr/...>` / `#include <ucdr/...>` resolving inside vendor/
    is treated the same (headers under include/ always land in the .h,
    even when first reached from a .c).
  - Any other `#include` (system headers, platform-guarded transports we
    did not vendor) is kept verbatim; those sit inside false `#ifdef`s
    under our baked config, exactly as upstream.
  - Placement is activity-aware (see below): a header first met inside
    dead (`#ifdef`-off) code is emitted again at its first live use, so
    every declaration also exists in live code. Include guards make the
    duplicate emission safe. All conditionals are kept verbatim; text is
    only relocated, never rewritten.

Activity model: a region is live iff the preprocessor would keep it in a
C TU built with the baked config. TRUE_DEFS holds the baked profiles;
`__cplusplus` counts as UNDEFINED (C-mode common denominator — vendor.h
serves C and C++ TUs, and no used declaration hides behind `#ifdef
__cplusplus` except `extern "C"` braces). Unknown/complex expressions
are assumed live; that can only add a harmless guard-deduped copy,
while the reverse (missing a live copy) would break the build.
`#define`s met in live code are learned, mirroring the output TU, so
`#ifdef`s on config-provided macros evaluate correctly.

Regenerate: `python3 vendor/amalgamate.py` from esphome/components/xrce_dds/
(or `python3 esphome/components/xrce_dds/vendor/amalgamate.py` from root).
The result must be byte-identical to the committed files
(tests/test_xrce_vendor.py checks this).
"""
import os
import re
import sys
from pathlib import Path

VENDOR = Path(__file__).parent
COMP = VENDOR.parent
UCR_INC = VENDOR / "microcdr" / "include"
UXR_INC = VENDOR / "microxrcedds" / "include"

INCLUDE_RE = re.compile(r'^\s*#\s*include\s*"([^"]+)"')
ANGLE_RE = re.compile(r"^\s*#\s*include\s*<([^>]+)>")
COND_RE = re.compile(r"^\s*#\s*(if|ifdef|ifndef|elif|else|endif)\b\s*(.*)$")
DEFINE_RE = re.compile(r"^\s*#\s*define\s+(\w+)")
UNDEF_RE = re.compile(r"^\s*#\s*undef\s+(\w+)")
DEFINED_RE = re.compile(r"defined\((\w+)\)")
IDENT_RE = re.compile(r"\w+")

# Baked profiles from vendor/*/include/*/config.h. Everything else
# UCLIENT_*/PLATFORM_*/WIN32/PERFORMANCE_TESTING counts as undefined.
TRUE_DEFS = frozenset({
    "UCLIENT_PROFILE_CUSTOM_TRANSPORT",
    "UCLIENT_PROFILE_STREAM_FRAMING",
    "UCLIENT_TWEAK_XRCE_WRITE_LIMIT",
})

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


# Quoted includes (by basename) that resolve inside vendor/ but were not
# vendored, and how to handle them. A kept line survives verbatim; this is
# only correct when the platform (ESP-IDF) or a false `#ifdef` provides it.
# A dropped line is proven dead. Anything else is a loud SystemExit so new
# upstream headers get a conscious decision instead of a broken build.
# - shared_memory_internal.h: DROPPED. Included unconditionally by
#   session.c/create_entities_{bin,xml}.c, but zero symbols from it are
#   referenced anywhere in the tree and UCLIENT_PROFILE_SHARED_MEMORY is
#   off in the baked config. (Kept verbatim it would break the build: the
#   amalgam lives at the component root where the relative path dies.)
# - FreeRTOS.h, semphr.h, task.h: KEPT. Guarded platform headers
#   (multithread.h: PLATFORM_NAME_FREERTOS, time.c: FREERTOS_PLUS_TCP),
#   both guards false in our config; IDF would provide them if true.
# Basename keying: these names are distinctive upstream; if upstream ever
# reuses one for a real vendored header, the freshness test still pins the
# exact output and the build will tell us.
DROP_BASENAMES = {"shared_memory_internal.h"}
KEEP_BASENAMES = {"FreeRTOS.h", "semphr.h", "task.h"}


class Amalgam:
    def __init__(self):
        self.state = {}  # resolved path -> "live" | "dead"
        self.known = set(TRUE_DEFS)
        self.h = []
        self.c = []

    def rel(self, path):
        return path.relative_to(VENDOR).as_posix()

    def eval_cond(self, expr):
        """C-mode defined-ness; unknown/complex reads as live (safe side)."""
        expr = expr.strip()
        if expr.startswith("defined(") and expr.endswith(")"):
            return expr[8:-1].strip() in self.known
        if expr.startswith("!defined(") and expr.endswith(")"):
            return expr[9:-1].strip() not in self.known
        if IDENT_RE.fullmatch(expr):
            return expr in self.known
        return True

    def emit_header(self, path, live):
        key = path.resolve()
        prev = self.state.get(key)
        if prev == "live" or (prev == "dead" and not live):
            return
        # Live re-emission after a dead one: include guards make the
        # duplicate safe, and only the live copy's declarations count.
        self.state[key] = "live" if live else "dead"
        self.h.append("/* === BEGIN %s === */\n" % self.rel(path))
        self._scan(path, live, self.h)
        self.h.append("/* === END %s === */\n" % self.rel(path))

    def emit_body(self, path, live):
        key = path.resolve()
        prev = self.state.get(key)
        if prev is not None:
            if not live or prev == "live":
                return
            # fallthrough: live re-emission of a dead-marked internal header
            self.state[key] = "live"
        else:
            self.state[key] = "live" if live else "dead"
        self.c.append("/* === BEGIN %s === */\n" % self.rel(path))
        self._scan(path, live, self.c)
        self.c.append("/* === END %s === */\n" % self.rel(path))

    def discover(self, path):
        """Pass 1: emit headers reachable from path (never the body)."""
        self._scan(path, True, None)

    def _scan(self, path, entry_live, buf):
        with open(path, "r", encoding="utf-8") as f:
            lines = f.readlines()
        live_stack = [entry_live]
        taken_stack = [entry_live]
        for line in lines:
            cond = COND_RE.match(line)
            if cond is not None:
                kind, expr = cond.group(1), cond.group(2)
                if kind in ("if", "ifdef", "ifndef"):
                    m = re.match(r"\s*(\w+)", expr)
                    name = m.group(1) if m else ""
                    if kind == "ifdef":
                        val = name in self.known
                    elif kind == "ifndef":
                        val = name not in self.known
                    else:
                        val = self.eval_cond(expr)
                    active = live_stack[-1] and val
                    live_stack.append(active)
                    taken_stack.append(active)
                elif kind in ("elif", "else"):
                    parent = live_stack[-2]
                    if kind == "else":
                        val = True
                    else:
                        val = self.eval_cond(expr)
                    active = parent and not taken_stack[-1] and val
                    live_stack[-1] = active
                    taken_stack[-1] = taken_stack[-1] or active
                else:  # endif
                    live_stack.pop()
                    taken_stack.pop()
                if buf is not None:
                    buf.append(line)
                continue
            live = live_stack[-1]
            dm = DEFINE_RE.match(line)
            if dm is not None:
                if live:
                    self.known.add(dm.group(1))
                if buf is not None:
                    buf.append(line)
                continue
            um = UNDEF_RE.match(line)
            if um is not None:
                if live:
                    self.known.discard(um.group(1))
                if buf is not None:
                    buf.append(line)
                continue
            inc = INCLUDE_RE.match(line)
            ang = ANGLE_RE.match(line) if inc is None else None
            if inc is not None:
                target = (path.parent / inc.group(1)).resolve()
                if target.is_file() and VENDOR.resolve() in target.parents:
                    if is_public(target):
                        self.emit_header(target, live)
                    else:
                        self.emit_body(target, live)
                    continue
                if VENDOR.resolve() in target.parents:
                    # Resolves inside vendor/ but was not vendored:
                    # drop proven-dead, keep platform-guarded, else
                    # fail loudly (see DROP/KEEP_BASENAMES above).
                    base = os.path.basename(os.fspath(target))
                    if base in DROP_BASENAMES:
                        if buf is not None:
                            buf.append("/* dropped (not vendored, zero references): %s */\n" % base)
                        continue
                    if base in KEEP_BASENAMES:
                        if buf is not None:
                            buf.append(line)
                        continue
                    raise SystemExit("unvendored include, update DROP/KEEP_BASENAMES or vendor it: %s (from %s)"
                                     % (inc.group(1), self.rel(path)))
                if buf is not None:
                    buf.append(line)
            elif ang is not None:
                target = map_angle(ang.group(1))
                if target is not None and target.is_file():
                    self.emit_header(target, live)
                    continue
                if buf is not None:
                    buf.append(line)
            elif buf is not None:
                buf.append(line)


def build():
    missing = [str(p) for p in SOURCES if not p.is_file()]
    if missing:
        raise SystemExit("missing vendored sources: %s" % missing)
    # Deduplicate SOURCES defensively: one entry per file, order kept.
    srcs = list(dict.fromkeys(SOURCES))
    am = Amalgam()
    # Seed with the umbrella headers so the amalgam is a complete
    # client.h/microcdr.h equivalent even for public headers no .c
    # includes directly (e.g. client.h itself is include-only).
    am.emit_header(UCR_INC / "ucdr" / "microcdr.h", True)
    am.emit_header(UXR_INC / "uxr" / "client" / "client.h", True)
    # Pass 1: headers reachable from the sources (bodies never emitted).
    for src in srcs:
        am.discover(src)
    # Pass 2: bodies + src-internal headers in upstream order.
    for src in srcs:
        am.emit_body(src, True)

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
