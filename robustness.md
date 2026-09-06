# Robustness Plan — ESPHome ↔ ROS 2 Bridge

Confirmed decisions: **static allocation only** (`std::array` / `StaticVector`, no heap post-`setup()`), **worker isolated to `xrce_dds/`** (no changes to `ros2/` threading model beyond draining queues).

Goal: production-grade declarative bridge for slow hobby robots dual-homed to Home Assistant (`api:`) and ROS 2 (`xrce_dds:`). Not demo-chasing. No deadlines.

Principles:
- Deterministic memory: compile-time bounds, single alloc at `setup()` at most, no `vector`/`map`/`deque`/`string` on hot path.
- ESPHome task contract: all `uxr_*` on worker; entities (`servo`/`switch`/`light`/`sensor`/`camera`) and `App.*` only on `loop()`.
- Copy across queues, never pointers to `sample_buf_` / stack temporaries / camera buffers.
- Bounded everything with drop policy + counters surfaced in `dump_config()` / logs.

Current baseline (see `README.md`, `AGENTS.md`):
- Core `ros2/`: 9 types (`Bool/Float32/Int32/String/JointState/JointTrajectory/ColorRGBA/Joy/CompressedImage`), `Joy` sub-only, `CompressedImage` pub-only push, `points[0]`-only trajectory, zeroed vel/eff, stamps `0`.
- Caps: `ROS2_MAX_JOINTS=16`, `TRAJ_POINTS=2`, `JOY 16/16`, `NAME 32`, `STRING 256`, `FRAME_ID 64`, `SUB/PUB 128`, `TARGETS 64`, `XRCE topics 16 / readers 8 / writers 8 / stream 2048 / history 4 / MTU 512`.
- Transports: `ros2_mqtt` (JSON debug) + `xrce_dds` (native CDR production, own non-blocking UDP socket / UART shim).
- Execution: 100% cooperative `loop()`; blocks up to ~1s in `uxr_create_session_retries` (`xrce_dds_component.cpp:201`) and `uxr_run_session_until_all_status` (`:368`, `ENTITY_TIMEOUT_MS=1000`); silent UDP blackhole known gap (`:145-147`).

## Phase 0 — Land baseline
Scope: `esphome/components/xrce_dds/*`, `examples/*.yaml`, `tests/test_xrce_dds_config.py`, tree hygiene.
- Commit `xrce_dds/` + examples + config test separately from future work. Remove `micro-cdr-unlicensed/` dup vs submodule, `.swp` artifact.
- Fix cheap debt: enforce `max_topics` (unchecked today), decide `Int32`/`String` dead-schema (codec exists, no target/source dispatch — allow or reject loudly), document truncation (`min(n,16)`, `strncpy`) and drop-on-disconnect.
- Tests: `python -m pytest tests/ -v` green; `esphome config` on all 4 examples green.
- Exit: clean `git status`, reproducible baseline.

## Phase 1 — Data-plane correctness
Scope: `ros2/__init__.py`, `ros2_types.h/.cpp`, `ros2_json.cpp`, `xrce_dds_codec.cpp`, `test_ros2_msg_parity.py`, examples.
- Time sync: SNTP → `builtin_interfaces/Time`; set `header.stamp`/`frame_id` on `JointState`, `CompressedImage` (today `0`/`""`). Subscribe `/clock` optional.
- New types in schema+struct+both codecs+parity: `geometry_msgs/Twist` (diff-drive in), `nav_msgs/Odometry` + `tf2_msgs/TFMessage` (odom/TF out), `sensor_msgs/{Imu,Range,BatteryState}`.
- Reliability: `last_rx_` liveliness timeout (fix blackhole gap), per-topic reliable/best-effort (today fixed XML), keep `MTU 512 × history 4` authoritative, `max_packet_length` warn-only.
- Docs: QoS/drop policy, `float64[]↔float[]`, name maps (`/x→rt/x`, `pkg/Type→pkg::msg::dds_::Type_`).
- Exit: RViz TF-validated rover (`cmd_vel` in, `odom`+TF out); parity green.

## Phase 2 — Concurrency foundation (no new ROS features)
Scope: `xrce_dds_component.h/cpp` (worker owns all `uxr_*`); `ros2_component.cpp` only gains queue drain.
- Worker: one `xTaskCreatePinnedToCore` (APP core, prio 5, 8–12kB). Moves `try_connect`, `create_pending_entities`, `pump_once`, `publish`, `publish_image`, `on_data_` session side.
- Queues (FreeRTOS, bounded, copy):
  - Outbound samples `loop→worker`: `{writer_idx, type, len, data[sizeof(JointTrajectoryMsg)]}` depth 8, drop-newest + `ESP_LOGW`.
  - Image mailbox: depth 1 drop-oldest, heap JPEG copy (Camera lifetime ends; never pass `get_data_buffer()` pointer).
  - Inbound `worker→loop`: `{reader_idx, type, len, data, rx_ms}` depth 8–16, drop-oldest + counter; `loop()` drains then calls existing `dispatch_*` (`ros2_component.cpp:154-329`) synchronously.
- Guards: only queues + atomic `link_`/`last_rx_` snapshot shared; `session_/streams_/bufs_/codec_/readers_/writers_` worker-exclusive; `subs_/pubs_/levels_` + all entity pointers loop-exclusive. Wake via `App.wake_loop_threadsafe()` / `enable_loop_soon_any_context()`; `xQueueReceive(portMAX_DELAY)` + `vTaskDelay(process_interval)` on worker, never spin.
- Rejected: `std::vector`/`std::queue+mutex` (fragmentation, `_M_realloc_insert` flash cost); use `StaticVector` + `xQueueCreate` (single bounded alloc at `setup`).
- Exit: no 1s stall on agent-down / image burst, TWDT clean, loop p99 measured, `dump_config()` shows drops/queue depths.

## Phase 3 — Generic service framework (no domain service yet)
Scope: `ros2_middleware.h` (`create_requester/replier`, timeout/cancel), `xrce_dds_codec` service codegen pattern, `ros2/__init__.py` `services:` schema mirroring sub/pub style, tests.
- One spike only: `std_srvs/Trigger` round-trip + timeout + agent-flap. Proves DDS-RPC `Requester`/`Replier`, `8/8` budget impact, reply fragmentation, static service slots (cap 4–8 concurrent, `StaticVector`).
- Exit: reusable req/res pattern; no navigation / trajectory actions yet.

## Phase 4 — Actions (spec now, build later)
- Spec only: goal/cancel/feedback/result mapping, preemption, feedback rate limits, `FollowJointTrajectory` single-setpoint + feedback shape. Requires Ph1 clocks + Ph2 queues + Ph3 RPC. No implementation in this plan.

## Phase 5 — HA duality + docs
- `api:` + `ros2:` coexistence example (e.g. light = HA `light` + `ColorRGBA`), entity↔topic semantic table, OTA-safe reconnect policy, README matrix update.
- Exit: one device, two citizens (HA entity + ROS topic) documented and demoed.

Risks / non-goals:
- WiFi jitter / agent RTT irreducible — fast control stays on host; MCU does slow actuation + telemetry (10–50 Hz best-effort).
- No hard-RT, safety cert, high-BW VIO/SLAM, raw `Image`/`PointCloud2` in this plan.
- Static caps stay: services 4–8 concurrent max; streams/topics as above.
