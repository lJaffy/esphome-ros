# AGENTS.md — ESPHome ROS 2 Custom Components

## Scope

Work only in: `esphome/components/ros2/`, `esphome/components/ros2_mqtt/`, `esphome/components/xrce_dds/`, `examples/*.yaml`, `tests/*.py`.

Do NOT touch, read for codegen, or document as source:

- `scratch/` (ignored PoC), `examples/.esphome/`, `__pycache__/`, `*.pyc`, `.pytest_cache/`, `build*/`, `.venv/`, `venv/`, `dist/`
- Submodule contents under `third_party/common_interfaces` and `esphome/components/xrce_dds/third_party/` (read-only vendored IDL / Micro-XRCE-DDS-Client / micro-CDR)

See root `.gitignore` + `examples/.gitignore` before adding files.

## Repo Map

- `esphome/components/ros2/__init__.py` — YAML schema + codegen (`CONFIG_SCHEMA`, `SUBSCRIPTION_SCHEMA`, `PUBLICATION_SCHEMA`, `to_code`). Single source of truth for validation rules.
- `ros2_component.{h,cpp}` — `Ros2Component`: sub/pub tables (16 each), `MiddlewareRegistry` lookup, `loop()` polling, `CameraListener` push.
- `ros2_types.{h,cpp}` — MCU structs + `TypeDef` table (`SUPPORTED_TYPES`); caps (`ROS2_MAX_*`, `ROS2_NAME_LEN=32`, `STRING_LEN=256`).
- `ros2_json.{h,cpp}` — `JsonCodec` serialize/deserialize (MQTT wire format).
- `ros2_middleware.{h,cpp}` — abstract `Ros2Middleware` + `MiddlewareRegistry` (max 4).
- `ros2_mqtt/` — `"mqtt"` middleware over `mqtt::CustomMQTTDevice`; base64 JPEG, never retained.
- `xrce_dds/` — `"xrce_dds"` middleware; `XrceDdsComponent`, `XcdrCodec`, `transport_udp` (own lwIP socket), `transport_serial` (UART shim). Caps: topics 16, readers/writers 8, stream 2048.
- `examples/` — 4 demos (joy lamp, stewart MQTT, stewart DDS, xiao camera). All use `external_components: {type: local, path: ../esphome/components/}`.
- `tests/` — `test_ros2_phase1.py` (schema), `test_xrce_dds_config.py` (schema), `test_ros2_msg_parity.py` (stdlib-only IDL parity).

## Commands

```bash
git submodule update --init --recursive
pip install -r requirements_dev.txt
python -m pytest tests/ -v
python -m pytest tests/test_ros2_msg_parity.py -v   # no esphome install needed
esphome config examples/<name>.yaml
esphome compile examples/<name>.yaml
```

No lint/typecheck config in repo; keep `python -m compileall` clean and match existing style.

## Conventions

### Python (`__init__.py`)

- Follow existing `cv.Schema` + `cv.All(validate)` pattern; put cross-field rules in `_validate_subscription` / `_validate_publication` / `_validate_transport`, not inline.
- Keep `SUPPORTED_TYPES` grouped: `SCALAR + MULTI_JOINT + LIGHT + IMAGE`. New types must update schema lists, C++ `TypeDef` table, both codecs, and parity test.
- `target:`/`targets:` and `source:`/`sources:` are mutually exclusive (exactly one). Multi-joint types require plural + unique `joint_name`.
- `_auto_load` must stay config-conditional (always `json`, add entity libs only if used). Full superset when called with no args.
- `to_code`: use `cg.get_variable` for entity IDs, `add_*_subscription|publication`, `set_middleware_name|default_publish_interval|status_sensor`.

### C++

- ESPHome style: `#pragma once`, `esphome::ros2` / `ros2_mqtt` / `xrce_dds` namespaces, `setup()/loop()/dump_config()/get_setup_priority()` overrides.
- Fixed-size `std::array`, no heap on hot path; shared `sample_buf_` sized to largest msg (`JointTrajectoryMsg`).
- Guard optional entity code with `USE_SERVO/LIGHT/CAMERA/etc.`; keep `AUTO_LOAD` and guards in sync.
- XRCE: `float64[]` on wire ↔ `float[]` on MCU; single-setpoint trajectory (`points[0]`); topic map `/x` → `rt/x`, type `pkg/Type` → `pkg::msg::dds_::Type_`. Never call blocking `uxr_delete_session` in destructor.
- UDP transport uses own POSIX socket (not `UDPComponent`); keep non-blocking + `MSG_DONTWAIT`, `EAGAIN → 0`.

### YAML examples

- Pin `esp32.board`, keep `logger:`, `wifi:` placeholders, broker `192.168.1.10`, `topic_prefix digitaltwin/*`.
- Prefer declarative `ros2:` subs/pubs; no lambdas. Comment ROS-side verify commands (`ros2 topic echo`, `MicroXRCEAgent udp4 -p 8888`).

## Adding a Message Type (checklist)

1. `ros2/__init__.py`: extend type list + target/source validation.
2. `ros2_types.h/.cpp`: struct + `TypeDef` entry (respect caps).
3. `ros2_json.cpp`: both directions; `ros2_mqtt.cpp` if special-cased (e.g. images).
4. `xrce_dds_codec.cpp`: XCDR size/serialize/deserialize + bounds checks.
5. `tests/test_ros2_msg_parity.py`: field expectations vs `third_party/common_interfaces`.
6. Example YAML + README row.

## Testing

- Schema tests need `esphome` installed (`pytest.importorskip`). Parity test must stay stdlib-only.
- Before PR: `python -m pytest tests/ -v` green, `esphome config` passes on touched examples.
- Never commit `secrets.yaml`, `.esphome/`, build artifacts, or PoC leftovers in `scratch/`.

## Git

- Never commit unless explicitly asked. Check `git status` for ignored-path leakage before committing.
- Submodules: `third_party/common_interfaces`, `xrce_dds/third_party/{Micro-XRCE-DDS-Client,micro-CDR}` — don't vendor copies elsewhere.
