# ESPHome ROS 2 Custom Components

Declarative ESPHome → ROS 2 bridge for microcontrollers. No lambdas required.

Three local custom components under `esphome/components/`:

| Component | Role | Middleware name | Deps |
|---|---|---|---|
| `ros2` | Core bridge: maps ESPHome entities to ROS 2 topics (pub/sub) | — (consumes `middleware:`) | `json` + conditional `sensor`, `switch`, `binary_sensor`, `servo`, `camera`, `light` |
| `ros2_mqtt` | JSON-over-MQTT transport (debug / fallback) | `mqtt` | `mqtt`, `ros2`, `json` |
| `xrce_dds` | Native XRCE-DDS transport via `micro-ros-agent` (production) | `xrce_dds` | `ros2`, `network` |

> Ignored and intentionally undocumented: `scratch/` (PoC), `examples/.esphome/`, `__pycache__/`, `.pytest_cache/`, `build/`, `.venv/`, `dist/`. See root `.gitignore` and `examples/.gitignore`.

## Requirements

- `esphome>=2026.6.0` (`requirements.txt`)
- `pytest>=8.0` for tests (`requirements_dev.txt`)
- Submodule: `third_party/common_interfaces` (ROS 2 IDL ground truth). XRCE-DDS C libraries (Micro-XRCE-DDS-Client v3.0.2 + micro-CDR v2.0.2) are vendored under `esphome/components/xrce_dds/vendor/` — no submodule init needed for DDS builds.

```bash
git submodule update --init --recursive
pip install -r requirements_dev.txt
```

## Install

All examples use local external components:

```yaml
external_components:
  - source:
      type: local
      path: ../esphome/components/
```

## `ros2`: core component

`Ros2Component` (`Component`, `CameraListener`, `AFTER_CONNECTION`) looks up `middleware:` by name in `MiddlewareRegistry`, retries every 1s, polls timed publications in `loop()`, pushes camera frames via `on_camera_image()`, and optionally drives a `status_sensor` from `connected()`.

Limits: 16 subscriptions, 16 publications, 16 targets/sources per topic, 16 joints, 2 trajectory points, 16 joy axes/buttons, name 32 B, string 256 B, frame_id 64 B.

### Use-cases

1. **Robot actuator (subscribe):** drive servos from `/joint_states` or `/joint_trajectory`, single `Float32` → one servo, `Bool` → switch.
2. **Sensor telemetry (publish):** `sensor` → `Float32`, `switch`/`binary_sensor` → `Bool`, servo positions → `JointState` echo.
3. **RGB signaling (bidirectional):** `ColorRGBA` ↔ `light`, `Joy` → `light` for gamepad control.
4. **Vision (publish-only push):** `camera` → `CompressedImage` (base64-in-JSON on MQTT by default, `use_b64: false` for raw JPEG bytes; fragmented CDR on XRCE).
5. **Link health:** `status_sensor` binary_sensor mirrors middleware `connected()`.

### Message types and entity bindings

| ROS 2 type | Subscribe `target:`/`targets:` | Publish `source:`/`sources:` |
|---|---|---|
| `std_msgs/Bool` | `switch` | `switch`, `binary_sensor` |
| `std_msgs/Float32` | `servo` (single) | `sensor` |
| `std_msgs/Int32`, `std_msgs/String` | — (rejected: codec-only, no entity mapping yet) | — (rejected) |
| `sensor_msgs/Range` | — (rejected: publish-only) | `sensor` + geometry opts (`radiation_type`, `field_of_view`, `min_range`, `max_range`, `variance`) |
| `sensor_msgs/BatteryState` | — (rejected: publish-only) | `sensor` (pack voltage) + `min_voltage`/`max_voltage` map, `design_capacity`, `technology`, `location` |
| `sensor_msgs/Imu` | — (rejected: publish-only) | `imu` (`accel_x/y/z` + `gyro_x/y/z`, optional `orientation_x/y/z/w`) |
| `geometry_msgs/Twist` | `diff_drive` (`left:`/`right:` wheel servos + geometry/scales) | — (rejected: subscribe-only) |
| `nav_msgs/Odometry` | — (rejected: publish-only) | `odom` (`wheel_separation` + `child_frame_id` + optional `tf_topic`) |
| `tf2_msgs/TFMessage` | — | `transforms:` static list (no `source:`), 1–4 entries |
| `sensor_msgs/JointState`, `trajectory_msgs/JointTrajectory` | `targets: [servo + joint_name]` | `sources: [servo + joint_name]` (trajectory uses `points[0]`) |
| `std_msgs/ColorRGBA` | `light (field: rgb\|brightness)` | `light` |
| `sensor_msgs/Joy` | `light (field: rgb)` — subscribe-only | — (rejected) |
| `sensor_msgs/CompressedImage` | — | `camera` |

`servo` mapping: `rad ↔ level -1..1` via `min_rad`/`max_rad` (default ∓π). `light rgb`: `r,g,b 0..1`, `a`/brightness; `Joy axes[0..2] → R,G,B`, `buttons[0]==0` = off.

Lossy by design, all bounded static allocation (no heap on hot path): joint names truncated to 31 chars + NUL, incoming joint lists clamped to 16, trajectory uses `points[0]` only with vel/eff zeroed, stamps are `0` (no clock sync yet). Publications are skipped while the middleware is disconnected (samples dropped, never queued); camera frames are dropped when throttled or offline.

### Full `ros2` reference

```yaml
ros2:
  middleware: mqtt  # required: "mqtt" | "xrce_dds" (MiddlewareRegistry name)
  default_publish_interval: 1s  # optional, per-publication `interval:` overrides
  status_sensor:
    platform: template  # optional, reflects middleware connected()
    name: ROS2 link
  subscriptions:
    # single-entity:
    - topic: /gripper/close
      type: std_msgs/Bool
      target:
        switch:
          id: gripper_switch
    - topic: /pan/command
      type: std_msgs/Float32
      target:
        servo:
          id: pan_servo
          field: position
          min_rad: -1.5708
          max_rad: 1.5708
    # multi-joint (joint_name required, unique):
    - topic: /joint_states
      type: sensor_msgs/JointState
      targets:
        - servo: {id: servo_1, joint_name: joint_1, min_rad: -1.5708, max_rad: 1.5708}
    # light:
    - topic: /led/color
      type: std_msgs/ColorRGBA
      target:
        light: {id: lamp, field: rgb}  # rgb | brightness
  publications:
    - topic: /temp
      type: std_msgs/Float32
      source: {sensor: {id: temp_sensor}}
      interval: 10s
    - topic: /joint_states_echo
      type: sensor_msgs/JointState
      sources:
        - servo: {id: servo_1, joint_name: joint_1}
      interval: 100ms
    - topic: /led/color_echo
      type: std_msgs/ColorRGBA
      source: {light: {id: lamp}}
    - topic: /camera/image/compressed
      type: sensor_msgs/CompressedImage
      source: {camera: {id: sense_camera}}
      # use_b64: false  # raw JPEG bytes on MQTT (default true = base64-in-JSON)
```

Validation: exactly one of `target:`/`targets:` and `source:`/`sources:`; multi-joint types require plural, all others singular. `Int32`/`String` and all publish-only types as subscriptions (`Joy`, `Range`, `BatteryState`, `Imu`) are rejected at validation, not silently dropped. `frame_id:` is rejected on headerless types; `use_b64:` only with `CompressedImage` (default `true`); `radiation_type:`/`field_of_view:`/`min_range:`/`max_range:`/`variance:` only with `Range`; `min_voltage:`/`max_voltage:`/`design_capacity:`/`technology:`/`location:` only with `BatteryState` (with `min ≤ max` cross-checks). `sensor_msgs/Imu` needs an `imu:` source with all six `accel_x/y/z` + `gyro_x/y/z` sensor refs; `orientation_x/y/z/w` must be all present or all absent.

### Time sync, stamps, and frames

```yaml
time:
  - platform: sntp
    id: sntp_time

ros2:
  middleware: xrce_dds
  time_id: sntp_time   # optional; stamps all headers, else zeros
  publications:
    - topic: /joint_states
      type: sensor_msgs/JointState
      frame_id: base_link  # optional, header types only, [A-Za-z0-9/_-]
      sources: [...]
```

`stamp.sec` comes from `RealTimeClock::utcnow()` (`nanosec` is 0 — ESPTime has no sub-second field); without `time:` (or before SNTP sync) stamps stay `0`. `frame_id` is truncated to 63 chars.

### QoS

Per-subscription/publication `qos: reliable | best_effort` (default `reliable`, i.e. current behavior):

```yaml
  subscriptions:
    - topic: /sonar_fix/target
      type: std_msgs/Float32
      qos: best_effort
      target: {servo: {id: pan_servo}}
```

MQTT maps explicit `reliable → qos 1`, `best_effort → qos 0`; unset keeps `ros2_mqtt.default_qos`. XRCE-DDS creates `BEST_EFFORT` endpoints on dedicated best-effort streams (own 2 kB output buffer; input needs none) and matching `BEST_EFFORT` endpoint QoS XML — reliable endpoints are byte-identical to before. Images always use the reliable fragmented stream. If your agent rejects the best-effort dialect, entity creation fails loudly in the logs (verify with `MicroXRCEAgent` first).

## `ros2_mqtt`: JSON-over-MQTT transport

Debug path: ROS 2 topics as JSON payloads on the ESPHome MQTT client. Registers `"mqtt"`.

```yaml
mqtt:
  broker: 192.168.1.10
  topic_prefix: digitaltwin/stewart

ros2_mqtt:
  topic_prefix: ""      # prepended to ROS topic
  default_qos: 0
  default_retain: false # images never retained
```

Images are hand-encoded `{"header":{...},"format":"jpeg","data":"<base64>"}` (no ArduinoJson arena blow-up).

## `xrce_dds`: native DDS transport

Production path: speaks XRCE-DDS to `micro-ros-agent`, appears as a real ROS 2 node. Registers `"xrce_dds"`. Own non-blocking UDP socket (lwIP) or UART shim; hand-written XCDR-LE codec, no heap on hot path.

```yaml
xrce_dds:
  agent_address: 192.168.1.10  # required, IPv4 literal
  agent_port: 8888
  transport:
    type: udp                 # udp | serial
    # uart_id: uart_bus       # required if serial
  domain_id: 0                # 0-255
  client_name: stewart
  process_interval: 10ms
  keepalive_timeout: 5s       # also reconnect backoff
  max_topics: 16
  max_datawriters: 8
  max_datareaders: 8
  # max_packet_length: 1472   # currently ignored, vendored MTU wins
```

Agent:

```bash
MicroXRCEAgent udp4 -p 8888
ros2 topic echo /joint_states
```

Caps: topics 16, readers/writers 8 each, stream buf 2048, history 4; image stream 44 kB / history 32 (~1408 B slots, one UDP datagram each — no IP fragmentation); vendored MTU 1472. `max_topics`/`max_datawriters`/`max_datareaders` are clamped at validation to those compile-time caps, and distinct DDS topics (shared across readers/writers) are budgeted at runtime — over-budget `subscribe`/`publish` fails loudly instead of overflowing. `dump_config` reports topics used, cumulative TX ok/fail and RX counts, per-reason image drops, and last-RX age (stale age with live link = silent-agent symptom, see known gap).

## Examples

All under `examples/` (run `esphome compile <file>` or Dashboard). Six demos: gamepad lamp, Stewart ×2, camera, telemetry, rover.

### 1. `joy_color_light.yaml` — gamepad RGB lamp (ESP32-S3, MQTT)

```yaml
esphome: {name: joy-light-demo, friendly_name: Joy Light Demo}
esp32: {board: esp32-s3-devkitc-1}
logger:
wifi: {ssid: wifi, password: wifi_password}
external_components:
  - source: {type: local, path: ../esphome/components/}
mqtt: {broker: 192.168.1.10, topic_prefix: digitaltwin/joy-light}
ros2_mqtt: {default_qos: 0}
output:
  - {platform: gpio, id: led_r, pin: GPIO4}
  - {platform: gpio, id: led_g, pin: GPIO5}
  - {platform: gpio, id: led_b, pin: GPIO6}
light:
  - platform: rgb
    id: lamp
    name: Demo Lamp
    red: led_r
    green: led_g
    blue: led_b
ros2:
  middleware: mqtt
  subscriptions:
    - topic: /led/color
      type: std_msgs/ColorRGBA
      target: {light: {id: lamp, field: rgb}}
    - topic: /joy
      type: sensor_msgs/Joy
      target: {light: {id: lamp, field: rgb}}
  publications:
    - topic: /led/color_echo
      type: std_msgs/ColorRGBA
      source: {light: {id: lamp}}
      interval: 1s
```

### 2. `stewart_jointstate.yaml` — 6-DOF Stewart via MQTT

ESP32-S3 + PCA9685 (`GPIO8/7`, 50 Hz, ch 0–5) + 6× servo. Replaces PoC lambda with name-based `JointState` matching.

```yaml
mqtt: {broker: 192.168.1.10, topic_prefix: digitaltwin/stewart}
ros2_mqtt: {default_qos: 0}
ros2:
  middleware: mqtt
  subscriptions:
    - topic: /joint_states
      type: sensor_msgs/JointState
      targets:
        - servo: {id: servo_1, joint_name: joint_1, min_rad: -1.5708, max_rad: 1.5708}
        - servo: {id: servo_2, joint_name: joint_2, min_rad: -1.5708, max_rad: 1.5708}
        - servo: {id: servo_3, joint_name: joint_3, min_rad: -1.5708, max_rad: 1.5708}
        - servo: {id: servo_4, joint_name: joint_4, min_rad: -1.5708, max_rad: 1.5708}
        - servo: {id: servo_5, joint_name: joint_5, min_rad: -1.5708, max_rad: 1.5708}
        - servo: {id: servo_6, joint_name: joint_6, min_rad: -1.5708, max_rad: 1.5708}
  publications:
    - topic: /joint_states_echo
      type: sensor_msgs/JointState
      sources:
        - servo: {id: servo_1, joint_name: joint_1, min_rad: -1.5708, max_rad: 1.5708}
        - servo: {id: servo_2, joint_name: joint_2, min_rad: -1.5708, max_rad: 1.5708}
      interval: 100ms
i2c: {sda: GPIO8, scl: GPIO7, scan: true}
pca9685: [{id: pca9685_hub1, frequency: 50}]
# + 6x pca9685 output + 6x servo (see file for full listing)
```

### 3. `stewart_xrce_dds.yaml` — same Stewart over XRCE-DDS

Identical servos, no `mqtt:`/`ros2_mqtt:`:

```yaml
esphome: {name: platform-dds, friendly_name: platform DDS}
esp32: {board: esp32-s3-devkitc-1}
logger:
api:
wifi: {ssid: wifi, password: wifi_password}
external_components:
  - source: {type: local, path: ../esphome/components/}
xrce_dds:
  agent_address: 192.168.1.10
  agent_port: 8888
  transport: {type: udp}
  domain_id: 0
  client_name: stewart
ros2:
  middleware: xrce_dds
  subscriptions:
    - topic: /joint_states
      type: sensor_msgs/JointState
      targets:
        - servo: {id: servo_1, joint_name: joint_1, min_rad: -1.5708, max_rad: 1.5708}
        # ... servo_2..6 identical (see file)
  publications:
    - topic: /joint_states_echo
      type: sensor_msgs/JointState
      sources:
        - servo: {id: servo_1, joint_name: joint_1, min_rad: -1.5708, max_rad: 1.5708}
        - servo: {id: servo_2, joint_name: joint_2, min_rad: -1.5708, max_rad: 1.5708}
      interval: 100ms
```

### 4. `xiao_sense_camera.yaml` — CompressedImage (XIAO Sense, MQTT)

```yaml
esphome: {name: xiao-sense-cam, friendly_name: Xiao Sense Cam}
esp32:
  board: seeed_xiao_esp32s3
  framework: {type: esp-idf}
logger:
wifi: {ssid: wifi, password: wifi_password}
external_components:
  - source: {type: local, path: ../esphome/components/}
mqtt: {broker: 192.168.1.10, topic_prefix: digitaltwin/xiao-sense}
ros2_mqtt: {default_qos: 0}
esp32_camera:
  id: sense_camera
  name: Sense Camera
  external_clock: {pin: GPIO10, frequency: 20MHz}
  i2c_pins: {sda: GPIO40, scl: GPIO39}
  data_pins: [GPIO15, GPIO17, GPIO18, GPIO16, GPIO14, GPIO12, GPIO11, GPIO48]
  vsync_pin: GPIO38
  href_pin: GPIO47
  pixel_clock_pin: GPIO13
  resolution: 1600x1200
  jpeg_quality: 14
  max_framerate: 1 fps
  idle_framerate: 1 fps
ros2:
  middleware: mqtt
  publications:
    - topic: /camera/image/compressed
      type: sensor_msgs/CompressedImage
      source: {camera: {id: sense_camera}}
      interval: 1s
```

Drop to XGA/SVGA if heap degrades at 1 fps.

### 5. `sensor_telemetry.yaml` — Range + BatteryState + Imu (ESP32-S3, MQTT)

Template sensors stand in for real drivers (`ultrasonic_sensor` in m, `adc` in V, `mpu6050` accel in m/s² + gyro in rad/s); SNTP time stamps all headers:

```yaml
time:
  - platform: sntp
    id: sntp_time
ros2:
  middleware: mqtt
  time_id: sntp_time
  publications:
    - topic: /sonar/range
      type: sensor_msgs/Range
      source: {sensor: {id: sonar_distance}}
      frame_id: sonar
      qos: best_effort
      radiation_type: ultrasound
      field_of_view: 0.5
      min_range: 0.02
      max_range: 4.0
      interval: 1s
    - topic: /battery/state
      type: sensor_msgs/BatteryState
      source: {sensor: {id: pack_voltage}}
      min_voltage: 3.0
      max_voltage: 4.2
      design_capacity: 2.5
      technology: lipo
      location: main_pack
      interval: 10s
    - topic: /imu/data
      type: sensor_msgs/Imu
      source:
        imu:
          accel_x: {id: imu_accel_x}
          accel_y: {id: imu_accel_y}
          accel_z: {id: imu_accel_z}
          gyro_x: {id: imu_gyro_x}
          gyro_y: {id: imu_gyro_y}
          gyro_z: {id: imu_gyro_z}
      frame_id: imu_link
      interval: 100ms
```

IMU semantics: accel in m/s², gyro in rad/s; skipped until all six axes have state. Orientation is optional (`orientation_x/y/z/w` quaternion sensors, normalized defensively with identity fallback); absent orientation publishes `0,0,0,0` with `orientation_covariance[0] = -1` ("no estimate") per the IDL. All other covariances publish as zeros (no uncertainty model — fuse on the host).

### 6. `gps_navsat.yaml` — NavSatFix (ESP32-S3, MQTT)

Template sensors stand in for a `gps:` receiver over UART (same sensor IDs); SNTP time stamps the header:

```yaml
ros2:
  middleware: mqtt
  time_id: sntp_time
  publications:
    - topic: /fix
      type: sensor_msgs/NavSatFix
      source:
        gps:
          latitude: {id: gps_latitude}
          longitude: {id: gps_longitude}
          altitude: {id: gps_altitude}  # optional; absent publishes NaN per the IDL
      frame_id: gps_antenna
      interval: 1s
```

Semantics: skipped until latitude + longitude both have state (reads as NO_FIX downstream); published samples carry `STATUS_FIX` + `SERVICE_GPS`. Covariance publishes as zeros with type UNKNOWN (no uncertainty model — fuse on the host for anything serious). Verify with `ros2 topic echo /fix`.

### 7. `rover_diff_drive.yaml` — cmd_vel rover with odometry + TF (ESP32-S3, XRCE-DDS)

Two continuous-rotation servos as wheel velocity outputs (`level -1..1` = reverse..forward):

```yaml
ros2:
  middleware: xrce_dds
  time_id: sntp_time
  subscriptions:
    - topic: /cmd_vel
      type: geometry_msgs/Twist
      target:
        diff_drive:
          left: {id: left_wheel}
          right: {id: right_wheel}
          wheel_separation: 0.2   # required, no default: silent geometry is worse than none
          max_linear_speed: 0.5
          max_angular_speed: 2.0
          cmd_timeout: 500ms      # stale cmd_vel zeroes the wheels (safety stop)
  publications:
    - topic: /odom
      type: nav_msgs/Odometry
      source:
        odom: {wheel_separation: 0.2}  # required, must match the base above
      frame_id: odom
      child_frame_id: base_link
      tf_topic: /tf               # also publish odom→base_link here (shares /tf with static below)
      interval: 100ms
    - topic: /tf
      type: tf2_msgs/TFMessage
      transforms:
        - frame_id: base_link
          child_frame_id: laser
          translation: [0.1, 0.0, 0.2]
          rotation: [0.0, 0.0, 0.0, 1.0]  # normalized defensively at runtime
      interval: 1s
```

Semantics, all documented limitations: planar only (`linear.y/z`, `angular.x/y` ignored); odometry is open-loop dead reckoning from the last commanded wheel velocity (integrates zero when `cmd_vel` goes stale — a silent base reads stopped, never drifting); covariances publish as zeros (no uncertainty model — fuse on the host for anything serious); `dt` clamps to 1 s across sleeps/reconnects. Verify with `ros2 topic echo /odom` and `ros2 run tf2_ros tf2_echo odom base_link`.

## Middleware choice

|  | `ros2_mqtt` | `xrce_dds` |
|---|---|---|
| Wire | JSON on MQTT | Bare CDR to `micro-ros-agent` |
| ROS 2 visibility | Needs bridge | Native node |
| Camera | base64 JSON, QoS 0 | Fragmented stream |
| Best for | Debug, dashboards | Production control |

## Tests

```bash
python -m pytest tests/test_ros2_phase1.py      # ros2 schema validation (needs esphome)
python -m pytest tests/test_xrce_dds_config.py  # xrce_dds schema (needs esphome)
python -m pytest tests/test_ros2_msg_parity.py  # stdlib-only IDL parity vs third_party/common_interfaces
python -m pytest tests/
```

## Repo layout

```text
esphome/components/ros2/       # __init__.py, ros2_component.{h,cpp}, ros2_{types,json,middleware}.{h,cpp}
esphome/components/ros2_mqtt/  # __init__.py, ros2_mqtt.{h,cpp}
esphome/components/xrce_dds/   # __init__.py, xrce_dds_{component,codec,transport_udp,transport_serial}.{h,cpp}, vendor/ (XRCE-DDS C libs, see vendor/VENDORED.md)
examples/*.yaml                # 7 demos above
tests/*.py                     # 3 pytest files
third_party/common_interfaces  # submodule, canonical .msg
```

## License

MIT © 2026 lJaffy. See `LICENSE`.
