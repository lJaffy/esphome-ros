#!/bin/sh
# Agent + recorder in one container (they share localhost DDS discovery).
#
# Default (no args): start `micro-ros-agent udp4 -p $AGENT_PORT` in the
# background, then run `capture.py record --out $OUT_DIR` in the foreground
# so Docker logs show the per-frame lines and Ctrl-C stops both.
# Any explicit command (e.g. `discover`, `echo`, `save`, `agent`) replaces
# the default: `exec "$@"` runs it instead (use with capture.py or the
# agent binary directly).
set -eu

OUT_DIR="${OUT_DIR:-/out}"
AGENT_PORT="${AGENT_PORT:-8888}"
DOMAIN="${ROS_DOMAIN_ID:-0}"
# Extra args appended to `capture.py record`, e.g.
#   -e CAPTURE_EXTRA_ARGS="--topic /joint_states_echo=JointState --stats-every 10"
CAPTURE_EXTRA_ARGS="${CAPTURE_EXTRA_ARGS:-}"

# ament/colcon setup scripts reference (possibly unset) vars like
# AMENT_TRACE_SETUP_FILES, which trips `set -u`. Source them with -u off.
ros_setup() {
  if [ -f /opt/ros/jazzy/setup.sh ]; then
    set +u
    # shellcheck disable=SC1091
    . /opt/ros/jazzy/setup.sh
    set -u
  fi
}

if [ "$#" -gt 0 ]; then
  case "$1" in
    trigger)
      # `trigger`: serve /toggle_led (std_srvs/Trigger) via rclpy.
      # Needs ROS_DOMAIN_ID matching the board's xrce_dds domain_id.
      shift
      ros_setup
      # shellcheck disable=SC2086
      exec python3 /opt/capture/trigger_server.py "$@"
      ;;
    with-trigger)
      # `with-trigger [extra record args...]`: agent + recorder + /toggle_led
      # server in one container (shares localhost DDS discovery). Extra args
      # are appended to `capture.py record` (plus CAPTURE_EXTRA_ARGS).
      shift
      ros_setup
      mkdir -p "$OUT_DIR"
      micro-ros-agent udp4 -p "$AGENT_PORT" &
      AGENT_PID=$!
      python3 /opt/capture/trigger_server.py &
      TRIGGER_PID=$!
      cleanup() {
        kill "$AGENT_PID" "$TRIGGER_PID" 2>/dev/null || true
      }
      trap cleanup INT TERM EXIT
      # shellcheck disable=SC2086
      python3 /opt/capture/capture.py record \
        --out "$OUT_DIR" \
        --domain "$DOMAIN" \
        $CAPTURE_EXTRA_ARGS "$@" &
      CAP_PID=$!
      wait "$CAP_PID"
      STATUS=$?
      cleanup
      trap - INT TERM EXIT
      exit "$STATUS"
      ;;
    *)
      exec "$@"
      ;;
  esac
fi

mkdir -p "$OUT_DIR"

micro-ros-agent udp4 -p "$AGENT_PORT" &
AGENT_PID=$!
cleanup() {
  kill "$AGENT_PID" 2>/dev/null || true
}
trap cleanup INT TERM EXIT

# Split CAPTURE_EXTRA_ARGS on whitespace (no shell quoting inside -e).
# shellcheck disable=SC2086
python3 /opt/capture/capture.py record \
  --out "$OUT_DIR" \
  --domain "$DOMAIN" \
  $CAPTURE_EXTRA_ARGS &
CAP_PID=$!
wait "$CAP_PID"
STATUS=$?
cleanup
trap - INT TERM EXIT
exit "$STATUS"
