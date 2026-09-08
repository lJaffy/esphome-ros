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

## Phase 1a — data-plane correctness: time, telemetry, QoS (done)
Scope: `ros2/__init__.py`, `ros2_types.h/.cpp`, `ros2_json.cpp`, `xrce_dds_codec.cpp`,
`ros2_mqtt`, `xrce_dds_component`, parity + schema tests, `examples/sensor_telemetry.yaml`.
- Time sync: optional `ros2.time_id: sntp_time` (`time::RealTimeClock`, inline `utcnow()`
  so no link dependency); stamps `JointState`/`Range`/`BatteryState`/`CompressedImage`
  (nanosec 0 — ESPTime has no sub-second field; zeros when unset/unsynced). Per-publication
  `frame_id:` (header types only, `[A-Za-z0-9/_-]` whitelist for the hand-encoded image JSON).
- New types `sensor_msgs/Range` (sensor + geometry opts) and `sensor_msgs/BatteryState`
  (voltage sensor + min/max percentage map, NaN for unmeasured, empty cell arrays/serial),
  publish-only (rejected as subscriptions, mirroring `Joy`). Schema + struct + both codecs +
  parity + `sensor_telemetry.yaml`.
- QoS: per-sub/pub `qos: reliable|best_effort` (default reliable = old behavior). MQTT maps
  explicit reliable→1/best_effort→0, unset keeps `default_qos`. XRCE-DDS creates BEST_EFFORT
  endpoints on dedicated best-effort streams (+2 kB static) with BEST_EFFORT endpoint QoS XML;
  images always stay reliable. Agent rejection of the dialect fails loudly.
- Reliability visibility (no behavior change): cumulative TX ok/fail + RX counters and last-RX
  age in `dump_config`; blackhole-agent gap still open (needs Phase 2 worker + ping design).
- Exit: `pytest tests/ -v` green, `esphome config` on all 5 examples green.

## Phase 1b — rover stack: Twist, Odometry, TF (done)
- `geometry_msgs/Twist` subscribe → `diff_drive:` target (left/right velocity servos,
  required `wheel_separation`, speed scales, `cmd_timeout` safety stop; planar only).
- `nav_msgs/Odometry` publish → open-loop dead reckoning from last commanded velocity
  (stale = zero, `dt` clamped, covariances zero) + `tf_topic:` bundling of odom→base_link.
- `tf2_msgs/TFMessage` publish → static `transforms:` list (1–4, quaternions normalized
  defensively); shares `/tf` with bundled odom frames. tf2_msgs has no IDL in the
  common_interfaces submodule (ros2/geometry2), so parity covers members/keys only.
- Large-sample fragmented DDS writes (Odometry ~716 B > 512 B history slot), best-effort
  streams keep whole-buffer writes.
- Exit: `pytest tests/ -v` green, `esphome config` on all 6 examples green, RViz
  TF-validated (`ros2 topic echo /odom`, `tf2_echo odom base_link`).

## Phase 1c — IMU (done)
- `sensor_msgs/Imu` publish → multi-sensor `imu:` source (`accel_x/y/z` +
  `gyro_x/y/z` required, `orientation_x/y/z/w` all-or-none optional,
  covariance -1/zero conventions).
- Exit: `pytest tests/ -v` green, `esphome config` on all 6 examples green,
  `ros2 topic echo /imu/data`.

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

Status (2026-09-08, validated on Seeed XIAO S3 camera @ 640x480, 1-3 fps over XRCE-DDS/UDP):

- Worker owns all `uxr_*` (connect, entity create, pump, publish, dispatch
  decode); `loop()` only enqueues. No 1 s stall observed; `api: Buffer full`
  seen once at boot, traced to DEBUG log traffic over the api socket, not
  loop starvation (power_save/modem-sleep tuning deferred by operator).
- TWDT clean across multi-minute runs; `PROBE fail 0` over 50+ frames at
  1 fps, sustained 3 fps without link loss.
- Loop cadence tracked in `Ros2Component` (`Loop max gap`, slow passes
  >100 ms in `dump_config`); worker stack HWM reported.
- `dump_config()` shows cumulative drops (TX fail, image
  link/no-writer/prepare/encode/mailbox-overwrite, inbound) plus live
  out/ctrl/mbox queue depths.
- Wire efficiency (Track 1, kept orthogonal to the worker): vendored MTU
  512->1472, image stream 64 kB/history-4 -> 44 kB/history-32 (~1408 B
  slots, no IP fragmentation), bulk image buffers on PSRAM heap with
  `USE_CAMERA` gating. Confirm latency p50 ~250 ms, max <1 s at 1 fps;
  single-frame drops (never link drops) under congestion.

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
