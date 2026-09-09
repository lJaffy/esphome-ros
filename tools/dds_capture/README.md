# dds_capture — XRCE-DDS capture for the ESP32-S3, no ROS 2 required

Runs `MicroXRCEAgent` and subscribes to the S3's DDS topics from plain
Python (Eclipse CycloneDDS). Subcommands: `agent`, `discover`, `echo`,
`save` (JPEGs to disk), `record` (all selected topics + `index.jsonl`).
Plus a `/toggle_led` Trigger server (Phase-3 dev exercise) via compose.

## S3 side

Point the firmware at this machine and match the domain:

```yaml
xrce_dds:
  agent_address: 192.168.1.10   # this machine's LAN IP
  agent_port: 8888
  transport: {type: udp}
  domain_id: 0                  # must match --domain (default 0)
```

## Docker (Ubuntu 24.04 host)

```bash
docker build -t s3-dds-capture tools/dds_capture

# Terminal 1: agent + recorder (S3 dials UDP :8888, frames land in ./runs)
mkdir -p runs
docker run --rm --net=host -v "$PWD/runs:/out" s3-dds-capture
ls runs/  # frame-*.jpg + index.jsonl

# Extra topics / tuning via env (domain comes from ROS_DOMAIN_ID):
docker run --rm --net=host -v "$PWD/runs:/out" \
    -e CAPTURE_EXTRA_ARGS="--topic /joint_states_echo=JointState --stats-every 10" \
    s3-dds-capture

# Single tools (override the default agent+record):
docker run --rm --net=host s3-dds-capture \
    python3 /opt/capture/capture.py discover
docker run --rm --net=host -v "$PWD/frames:/out" s3-dds-capture \
    python3 /opt/capture/capture.py save --out /out
file frames/*.jpg | head
```

## Trigger service (Phase-3 dev exercise)

The agent alone cannot reply to the board's `/toggle_led` service client
(it only bridges XRCE to DDS). `trigger_server.py` is a minimal `rclpy`
`std_srvs/Trigger` server on fixed `/toggle_led` for local HIL:

```bash
docker compose -f tools/dds_capture/docker-compose.yml up --build
# or single container: agent + recorder + server together
docker run --rm --net=host -v "$PWD/runs:/out" s3-dds-capture with-trigger
```

`ROS_DOMAIN_ID` must match the board's `xrce_dds: domain_id:` (0).
Verify from the host: `ros2 service list | grep toggle_led`.
On the board, toggling the switch should then log
`Service /toggle_led reply success=1` instead of `failed (timeout=1)`.

`--net=host` is Linux-only and avoids mapping UDP ports. `ROS_DOMAIN_ID`
is 0 in the image; override with `-e ROS_DOMAIN_ID=0 --domain 0`.

## Native (no docker)

```bash
# Agent: prebuilt `microros/micro-ros-agent` image is easiest; natively use
# the micro-ROS snap (`snap install micro-ros-agent --classic`) or build
# https://github.com/eProsima/Micro-XRCE-DDS-Agent from a fresh tag —
# note v2.4.x source builds are broken (pinned Fast-DDS branch 2.12.x
# no longer resolves upstream).
micro-ros-agent udp4 -p 8888

pip install -r tools/dds_capture/requirements.txt
python3 tools/dds_capture/capture.py discover
python3 tools/dds_capture/capture.py save --out ./frames
```

## Usage

```bash
capture.py agent [--port 8888] [-- <extra agent args>]
capture.py discover [--wait 5] [--topic /t] [--domain 0]
capture.py echo [--topic /camera/image/compressed] [--stats-every 5]
capture.py save --out ./frames [--topic ...]
capture.py record --out ./run1 [--topic /a] [--topic /b=JointState]
```

`--topic` repeats as `NAME` (known S3 default) or `NAME=Type`, e.g.
`/joint_states_echo=JointState`. Supported types: every type the ros2
bridge can emit — `Bool`, `Float32`, `Int32`, `String`, `CompressedImage`,
`JointState`, `JointTrajectory`, `ColorRGBA`, `Joy`, `Range`,
`BatteryState`, `Twist`, `Odometry`, `TFMessage`, `Imu`, `NavSatFix`.
`save` only handles image topics. `record` writes one run dir: validated
`frame-NNNNNN_<UTC>.jpg` files (non-JPEG payloads are dropped and counted)
plus `index.jsonl` (one JSON object per line: `run_start` header, then
per-sample `t_wall`/`n`/`topic`/`type` with `file`/`bytes`/stamps for
images, `repr` for everything else) — convertible to a rosbag once ROS 2
exists.

## Gotchas

- **Silent no-data is almost always a name mismatch.** The S3 registers
  DDS topics as `rt/...` (`/camera/image/compressed` →
  `rt/camera/image/compressed`) with suffixed type names
  (`sensor_msgs::msg::dds_::CompressedImage_`). This tool mirrors
  `xrce_dds_codec.cpp`; if the codec changes, update `dds_types.py`
  (covered by `tests/test_dds_capture.py`).
- Domain mismatch (`--domain` vs S3 `domain_id:`) also gives silence.
- UDP + host firewall: allow inbound `8888/udp` (docker `--net=host`
  bypasses this).
- The S3 must actually be publishing: `discover` distinguishes "no
  publisher" (nothing matched) from "wrong type" (reader exists, no data).
