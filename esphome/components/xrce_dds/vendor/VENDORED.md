# Vendored XRCE-DDS sources

ESPHome `external_components` fetches a git repo **without** initializing
submodules, so the add-on build never sees
`third_party/Micro-XRCE-DDS-Client` or `third_party/micro-CDR` contents
(empty dirs → `ucdr/microcdr.h: No such file`). Even with submodules,
the full upstream trees cannot compile under ESPHome:

- ~30 `int main()` files under `examples/`/`test/` would all be glob-compiled into
  one component → duplicate-`main` link failure.
- Two CMake-generated headers are absent (`ucdr/config.h`,
  `uxr/client/config.h` — only `.in` templates ship).
- ~60 `src/*.c` files pull unneeded profiles (serial → `termios.h`,
  absent on ESP-IDF) while this component only uses the custom transport.

This `vendor/` tree is the minimal self-contained subset that builds.

## Why an amalgamation

Two ESPHome ESP-IDF limits force the committed amalgamation
(`../xrce_dds_vendor.{h,c}`) on top of this tree:

- Only files directly in the component directory are staged into the
  build (`loader.py::resources`, no recursion), so a deep `vendor/` tree
  is never copied and its 30 `.c` files never compile.
- Only `-D`/`-W` build flags reach CMake
  (`framework_helpers.py::get_project_compile_flags`), so `-I` include
  dirs are silently dropped.

`xrce_dds_vendor.h` (all public headers inlined) +
`xrce_dds_vendor.c` (all bodies + src-internal headers, compiled as C)
need no extra include dirs and build from the component root on both
ESP-IDF and Arduino. Our code includes only `"xrce_dds_vendor.h"`.

## Contents

- `microxrcedds/` — Micro-XRCE-DDS-Client **v3.0.2**, profile =
  custom-transport + stream framing (serial) only.
  - 30 `.c`: core session/streams/serialization, `util/time.c`,
    `util/ping.c`, `core/log/log.c`, `profile/transport/custom/custom_transport.c`,
    `profile/transport/stream_framing/stream_framing_protocol.c`,
    plus co-located `*_internal.h` headers those `.c` files include.
  - Headers: `core/`, `util/`, custom, stream-framing, multithread stub
    (no-ops with multithread off), top-level `client.h`/`transport.h`/
    `defines.h`/`visibility.h`.
  - `include/uxr/client/config.h` is generated (baked from CMake defaults:
    1 in/out × best-effort/reliable stream, attempts 10, interval 1000 ms,
    heartbeat 100 ms, custom MTU 512, `UCLIENT_TWEAK_XRCE_WRITE_LIMIT` on).
- `microcdr/` — micro-CDR **v2.0.2**, little-endian (`config.h` baked with
  `UCDR_MACHINE_ENDIANNESS = UCDR_LITTLE_ENDIANNESS`).
  - 5 `.c`: `common.c` + `types/{array,basic,sequence,string}.c`.
- `microxrcedds/LICENSE`, `microcdr/LICENSE` — upstream Apache-2.0 texts.

Deliberately excluded: `examples/`, `test/`, discovery, UDP/TCP/serial/CAN
platform transports (`ip_*.c`, `*_posix.c`, `*_windows.c`, …), `matching.c`,
`multithread.c`, `shared_memory.c`, and all platform-specific headers.

## Regenerating

```bash
python3 vendor/amalgamate.py   # from esphome/components/xrce_dds/
```

This rewrites `../xrce_dds_vendor.h` + `../xrce_dds_vendor.c`
byte-deterministically from the tree below
(`tests/test_xrce_vendor.py::test_amalgamation_fresh` enforces it —
never hand-edit the generated files). Rules: quoted/uxr/ucdr includes
resolving inside `vendor/` are inlined once with `BEGIN/END` markers
(public headers → `.h`, bodies + src-internal headers → `.c`); anything
else (system headers, platform-guarded transports) is kept verbatim.
Two named exceptions, both explicit in `amalgamate.py`: `DROP_BASENAMES`
(`shared_memory_internal.h` — included unconditionally but zero
references, profile off; kept verbatim it would not resolve from the
component root) is dropped with a comment, `KEEP_BASENAMES`
(`FreeRTOS.h`, `semphr.h`, `task.h` — guarded platform headers provided
by IDF when their guards are true) is kept; any other unvendored include
fails the script loudly so new upstream headers get a conscious decision.

Bump `UXR_CLIENT_VERSION_*` / `MICROCDR_VERSION_*` in the baked headers
when moving to a new upstream tag, and re-copy `LICENSE` files.
