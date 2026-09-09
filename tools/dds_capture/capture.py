#!/usr/bin/env python3
"""Capture XRCE-DDS data from the ESP32-S3 without a ROS 2 install.

Needs MicroXRCEAgent (binary, docker image provides it) and:
    pip install -r requirements.txt   # eclipse-cyclonedds

All DDS imports are lazy so `--help` and the unit tests work anywhere.

Examples:
    capture.py agent --port 8888
    capture.py echo --topic /camera/image/compressed
    capture.py save --topic /camera/image/compressed --out ./frames
    capture.py record --topic /camera/image/compressed=CompressedImage \\
        --topic /joint_states_echo=JointState --out ./run1
    capture.py discover --topic /camera/image/compressed
"""

import argparse
import json
import os
import shutil
import signal
import subprocess
import sys
import time

from dds_types import (
    DEFAULT_TOPICS,
    ROS_TO_DDS_TYPE,
    dds_topic_name,
    frame_filename,
    is_jpeg,
    utcnow,
)

# micro-ros-agent wraps MicroXRCEAgent with the same `udp4 -p` CLI and is
# what this repo's Dockerfile ships (upstream source builds are broken).
AGENT_BIN_CANDIDATES = ("micro-ros-agent", "micro_ros_agent",
                        "MicroXRCEAgent", "micro-xrce-dds-agent")

# Short ROS types accepted after "--topic NAME=Type". Covers every type the
# ros2 bridge can emit (see ROS_TO_DDS_TYPE in dds_types.py, mirrored from
# xrce_dds_codec.cpp). Fixed float64 arrays use array[float64, N] (bare
# elements on the wire, no length prefix); sequences are uint32 length +
# elements.
SUPPORTED_TYPES = ("Bool", "Float32", "Int32", "String", "CompressedImage",
                   "JointState", "JointTrajectory", "ColorRGBA", "Joy",
                   "Range", "BatteryState", "Twist", "Odometry",
                   "TFMessage", "Imu", "NavSatFix")

# Short type -> fully-qualified ROS type (std_msgs/Bool and friends do NOT
# live under sensor_msgs/, so make_readers must not guess the package).
SHORT_TO_ROS = {k.split("/")[-1]: k for k, v in ROS_TO_DDS_TYPE.items()}


def parse_topic_arg(arg):
    """'NAME' or 'NAME=Type' -> (ros_topic, short_type)."""
    if "=" in arg:
        name, typ = arg.split("=", 1)
    else:
        name, typ = arg, None
    name = name.strip()
    if not name.startswith("/"):
        raise ValueError("topic must start with '/': %r" % arg)
    if typ is not None:
        typ = typ.strip()
        if "/" in typ:  # allow fully-qualified sensor_msgs/CompressedImage
            short = typ.split("/")[-1]
        else:
            short = typ
        if short not in SUPPORTED_TYPES:
            raise ValueError("unsupported type %r (need one of %s)"
                             % (typ, ", ".join(SUPPORTED_TYPES)))
        return name, short
    # Default: look up the S3's own default publication map.
    if name not in DEFAULT_TOPICS:
        raise ValueError("no default type for %r: use NAME=Type" % name)
    return name, DEFAULT_TOPICS[name].split("/")[-1]


def resolve_topics(args):
    """CLI --topic list (or defaults) -> [(ros_topic, short_type)]."""
    if not args.topic:
        return [(t, DEFAULT_TOPICS[t].split("/")[-1]) for t in DEFAULT_TOPICS]
    return [parse_topic_arg(a) for a in args.topic]


def find_agent_bin():
    for cand in AGENT_BIN_CANDIDATES:
        path = shutil.which(cand)
        if path:
            return path
    return None


def cmd_agent(args):
    agent = args.agent_bin or find_agent_bin()
    if agent is None:
        print("No agent binary found (tried: %s). This repo's Dockerfile "
              "ships micro-ros-agent; otherwise pass --agent-bin PATH."
              % ", ".join(AGENT_BIN_CANDIDATES), file=sys.stderr)
        return 2
    cmd = [agent, "udp4", "-p", str(args.port)] + (args.agent_args or [])
    print("+ " + " ".join(cmd), flush=True)
    try:
        proc = subprocess.Popen(cmd)
    except OSError as exc:
        print("failed to start agent: %s" % exc, file=sys.stderr)
        return 1

    def _stop(*_):
        proc.terminate()

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)
    return proc.wait()


# --- DDS (lazy) -----------------------------------------------------------

def _dds():
    try:
        from cyclonedds.domain import DomainParticipant  # noqa
        from cyclonedds.topic import Topic  # noqa
        from cyclonedds.sub import DataReader  # noqa
        from cyclonedds.idl import IdlStruct  # noqa
        from cyclonedds.idl.types import int32, uint32, uint8, float64, sequence  # noqa
        import cyclonedds  # noqa
    except ImportError:
        print("cyclonedds not installed: pip install -r requirements.txt",
              file=sys.stderr)
        raise SystemExit(2)
    import cyclonedds.domain
    import cyclonedds.topic
    import cyclonedds.sub
    import cyclonedds.idl
    import cyclonedds.idl.types
    return (cyclonedds.domain.DomainParticipant,
            cyclonedds.topic.Topic,
            cyclonedds.sub.DataReader,
            cyclonedds.idl.IdlStruct,
            cyclonedds.idl.types)


def _idl_types():
    """Runtime IDL mirroring the S3's CDR layout (little-endian on the wire).

    Member order matches xrce_dds_codec.cpp serialize() field-for-field
    (CDR is positional). Fixed arrays use array[float64, N] (bare doubles);
    float32/int32/uint32/bool are their CDR widths.
    """
    from dataclasses import dataclass
    _, _, _, IdlStruct, idlt = _dds()
    int32, uint32, uint8, float64 = idlt.int32, idlt.uint32, idlt.uint8, idlt.float64
    sequence = idlt.sequence
    int8 = idlt.int8
    uint16 = idlt.uint16
    float32 = idlt.float32
    array = idlt.array
    boolean = getattr(idlt, "boolean", bool)

    @dataclass
    class Time(IdlStruct):
        sec: int32
        nanosec: uint32

    @dataclass
    class Duration(IdlStruct):
        sec: int32
        nanosec: uint32

    @dataclass
    class Header(IdlStruct):
        stamp: Time
        frame_id: str

    @dataclass
    class Vector3(IdlStruct):
        x: float64
        y: float64
        z: float64

    @dataclass
    class Quaternion(IdlStruct):
        x: float64
        y: float64
        z: float64
        w: float64

    @dataclass
    class Point(IdlStruct):
        x: float64
        y: float64
        z: float64

    @dataclass
    class Pose(IdlStruct):
        position: Point
        orientation: Quaternion

    @dataclass
    class Twist(
            IdlStruct,
            typename=ROS_TO_DDS_TYPE["geometry_msgs/Twist"]):
        linear: Vector3
        angular: Vector3

    @dataclass
    class PoseWithCovariance(IdlStruct):
        pose: Pose
        covariance: array[float64, 36]

    @dataclass
    class TwistWithCovariance(IdlStruct):
        twist: Twist
        covariance: array[float64, 36]

    @dataclass
    class Transform(IdlStruct):
        translation: Vector3
        rotation: Quaternion

    @dataclass
    class TransformStamped(IdlStruct):
        header: Header
        child_frame_id: str
        transform: Transform

    @dataclass
    class NavSatStatus(IdlStruct):
        status: int8
        service: uint16

    @dataclass
    class JointTrajectoryPoint(IdlStruct):
        positions: sequence[float64]
        velocities: sequence[float64]
        accelerations: sequence[float64]
        effort: sequence[float64]
        time_from_start: Duration

    @dataclass
    class Bool(
            IdlStruct,
            typename=ROS_TO_DDS_TYPE["std_msgs/Bool"]):
        data: boolean

    @dataclass
    class Float32(
            IdlStruct,
            typename=ROS_TO_DDS_TYPE["std_msgs/Float32"]):
        data: float32

    @dataclass
    class Int32(
            IdlStruct,
            typename=ROS_TO_DDS_TYPE["std_msgs/Int32"]):
        data: int32

    @dataclass
    class String(
            IdlStruct,
            typename=ROS_TO_DDS_TYPE["std_msgs/String"]):
        data: str

    @dataclass
    class ColorRGBA(
            IdlStruct,
            typename=ROS_TO_DDS_TYPE["std_msgs/ColorRGBA"]):
        r: float32
        g: float32
        b: float32
        a: float32

    @dataclass
    class CompressedImage(
            IdlStruct,
            typename=ROS_TO_DDS_TYPE["sensor_msgs/CompressedImage"]):
        header: Header
        format: str
        data: sequence[uint8]

    @dataclass
    class JointState(
            IdlStruct,
            typename=ROS_TO_DDS_TYPE["sensor_msgs/JointState"]):
        header: Header
        name: sequence[str]
        position: sequence[float64]
        velocity: sequence[float64]
        effort: sequence[float64]

    @dataclass
    class JointTrajectory(
            IdlStruct,
            typename=ROS_TO_DDS_TYPE["trajectory_msgs/JointTrajectory"]):
        header: Header
        joint_names: sequence[str]
        points: sequence[JointTrajectoryPoint]

    @dataclass
    class Joy(
            IdlStruct,
            typename=ROS_TO_DDS_TYPE["sensor_msgs/Joy"]):
        header: Header
        axes: sequence[float32]
        buttons: sequence[int32]

    @dataclass
    class Range(
            IdlStruct,
            typename=ROS_TO_DDS_TYPE["sensor_msgs/Range"]):
        header: Header
        radiation_type: uint8
        field_of_view: float32
        min_range: float32
        max_range: float32
        range: float32
        variance: float32

    @dataclass
    class BatteryState(
            IdlStruct,
            typename=ROS_TO_DDS_TYPE["sensor_msgs/BatteryState"]):
        header: Header
        voltage: float32
        temperature: float32
        current: float32
        charge: float32
        capacity: float32
        design_capacity: float32
        percentage: float32
        power_supply_status: uint8
        power_supply_health: uint8
        power_supply_technology: uint8
        present: boolean
        cell_voltage: sequence[float32]
        cell_temperature: sequence[float32]
        location: str
        serial_number: str

    @dataclass
    class Odometry(
            IdlStruct,
            typename=ROS_TO_DDS_TYPE["nav_msgs/Odometry"]):
        header: Header
        child_frame_id: str
        pose: PoseWithCovariance
        twist: TwistWithCovariance

    @dataclass
    class TFMessage(
            IdlStruct,
            typename=ROS_TO_DDS_TYPE["tf2_msgs/TFMessage"]):
        transforms: sequence[TransformStamped]

    @dataclass
    class Imu(
            IdlStruct,
            typename=ROS_TO_DDS_TYPE["sensor_msgs/Imu"]):
        header: Header
        orientation: Quaternion
        orientation_covariance: array[float64, 9]
        angular_velocity: Vector3
        angular_velocity_covariance: array[float64, 9]
        linear_acceleration: Vector3
        linear_acceleration_covariance: array[float64, 9]

    @dataclass
    class NavSatFix(
            IdlStruct,
            typename=ROS_TO_DDS_TYPE["sensor_msgs/NavSatFix"]):
        header: Header
        status: NavSatStatus
        latitude: float64
        longitude: float64
        altitude: float64
        position_covariance: array[float64, 9]
        position_covariance_type: uint8

    return {"Bool": Bool, "Float32": Float32, "Int32": Int32,
            "String": String, "CompressedImage": CompressedImage,
            "JointState": JointState, "JointTrajectory": JointTrajectory,
            "ColorRGBA": ColorRGBA, "Joy": Joy, "Range": Range,
            "BatteryState": BatteryState, "Twist": Twist,
            "Odometry": Odometry, "TFMessage": TFMessage, "Imu": Imu,
            "NavSatFix": NavSatFix}


def make_readers(domain_id, topics):
    """topics: [(ros_topic, short_type)] -> (participant, [(ros_topic, reader)]).

    Keep the participant alive as long as the readers (callers must hold
    it — dropping it destroys the readers)."""
    DomainParticipant, Topic, DataReader, _, _ = _dds()
    idl = _idl_types()
    dp = DomainParticipant(domain_id)
    readers = []
    for ros_topic, short in topics:
        ros_type = SHORT_TO_ROS[short]
        dds_type = ROS_TO_DDS_TYPE[ros_type]
        topic = Topic(dp, dds_topic_name(ros_topic), idl[short])
        readers.append((ros_topic, DataReader(dp, topic)))
        print("listening: %s  (dds %s / %s)" %
              (ros_topic, dds_topic_name(ros_topic), dds_type), flush=True)
    return dp, readers


def take_samples(reader):
    """Non-blocking drain; returns [deserialized, ...]."""
    try:
        samples = reader.take()
    except Exception:
        return []
    out = []
    for sample in samples:
        # reader.take() yields payload objects directly in this cyclonedds
        # version; older wrappers yield Sample objects with .data +
        # .sample_info. Only unwrap the latter: CompressedImage itself has
        # a `.data` field (the JPEG bytes), so blind getattr(..., "data")
        # would return just the byte array instead of the message.
        if hasattr(sample, "sample_info") and hasattr(sample, "data"):
            data = sample.data
        else:
            data = sample
        if data is not None:
            out.append(data)
    return out


# --- subcommands ----------------------------------------------------------

def cmd_discover(args):
    topics = resolve_topics(args)
    _dp, readers = make_readers(args.domain, topics)  # noqa: F841 (keeps readers alive)
    print("waiting %.1fs for publishers..." % args.wait, flush=True)
    deadline = time.monotonic() + args.wait
    seen = {t: 0 for t, _ in topics}
    while time.monotonic() < deadline:
        for ros_topic, reader in readers:
            seen[ros_topic] += len(take_samples(reader))
        time.sleep(0.1)
    rc = 0
    for ros_topic, _ in topics:
        if seen[ros_topic]:
            print("MATCHED %-32s %d sample(s)" % (ros_topic, seen[ros_topic]))
        else:
            print("NO DATA %-32s (S3 publishing? type-name match? "
                  "domain %d?)" % (ros_topic, args.domain))
            rc = 1
    return rc


def _stats_loop(readers, on_sample, args):
    counts = {t: 0 for t, _ in readers}
    start = time.monotonic()
    last_log = start
    try:
        while True:
            for ros_topic, reader in readers:
                for sample in take_samples(reader):
                    counts[ros_topic] += 1
                    on_sample(ros_topic, sample, counts[ros_topic])
            now = time.monotonic()
            if args.stats_every and now - last_log >= args.stats_every:
                el = now - start
                msg = "  ".join("%s: %d (%.1f/s)" % (t, c, c / el)
                                for t, c in counts.items())
                print("[%.0fs] %s" % (el, msg), flush=True)
                last_log = now
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    return 0


def cmd_echo(args):
    _dp, readers = make_readers(args.domain, resolve_topics(args))  # noqa: F841 (keeps readers alive)
    if args.stats_every is None:
        args.stats_every = 5.0

    def _show(ros_topic, sample, n):
        kind = type(sample).__name__
        if kind == "CompressedImage":
            print("%s #%d stamp=%d.%09d frame='%s' %d bytes" % (
                ros_topic, n, sample.header.stamp.sec,
                sample.header.stamp.nanosec, sample.header.frame_id,
                len(sample.data)))
        else:
            print("%s #%d %s" % (ros_topic, n, sample))

    return _stats_loop(readers, _show, args)


def cmd_save(args):
    topics = [t for t in resolve_topics(args) if t[1] == "CompressedImage"]
    if not topics:
        print("save needs an image topic (got none).", file=sys.stderr)
        return 2
    os.makedirs(args.out, exist_ok=True)
    _dp, readers = make_readers(args.domain, topics)  # noqa: F841 (keeps readers alive)
    if args.stats_every is None:
        args.stats_every = 5.0
    state = {"n": 0, "dropped": 0}

    def _save(ros_topic, sample, _n):
        raw = bytes(bytearray(sample.data))
        if not is_jpeg(raw):
            state["dropped"] += 1
            print("dropped non-JPEG frame on %s (%d bytes)" % (ros_topic, len(raw)))
            return
        state["n"] += 1
        path = os.path.join(
            args.out, os.path.basename(
                frame_filename("", utcnow(), state["n"])))
        with open(path, "wb") as fh:
            fh.write(raw)
        print("wrote %s (%d bytes)" % (path, len(raw)), flush=True)

    rc = _stats_loop(readers, _save, args)
    print("saved %d frames, dropped %d" % (state["n"], state["dropped"]))
    return rc


def cmd_record(args):
    topics = resolve_topics(args)
    os.makedirs(args.out, exist_ok=True)
    index_path = os.path.join(args.out, "index.jsonl")
    _dp, readers = make_readers(args.domain, topics)  # noqa: F841 (keeps readers alive)
    if args.stats_every is None:
        args.stats_every = 5.0
    index_fh = open(index_path, "a")
    index_fh.write(json.dumps({
        "event": "run_start",
        "t_wall": time.time(),
        "domain": args.domain,
        "topics": [{"topic": t, "type": s,
                     "dds_topic": dds_topic_name(t),
                     "dds_type": ROS_TO_DDS_TYPE["sensor_msgs/" + s]}
                   for t, s in topics],
    }) + "\n")
    index_fh.flush()
    state = {"saved": 0, "dropped": 0, "other": 0}

    def _rec(ros_topic, sample, n):
        try:
            # take() sometimes yields the bare byte array (dynamic typing
            # when the agent's type description wins over ours): unwrap
            # (data, info) tuples, then treat list/bytes as raw JPEG.
            payload = sample
            if isinstance(payload, tuple) and len(payload) == 2:
                payload = payload[0]
            kind = type(sample).__name__
            entry = {"t_wall": time.time(), "n": n, "topic": ros_topic,
                     "type": kind}
            if isinstance(payload, (list, bytes, bytearray)):
                raw = bytes(bytearray(payload))
                entry.update({"bytes": len(raw), "jpeg": is_jpeg(raw),
                              "raw_delivery": True})
                if not is_jpeg(raw):
                    state["dropped"] += 1
                    entry["dropped"] = True
                    print("dropped non-JPEG frame on %s (%d bytes)"
                          % (ros_topic, len(raw)), flush=True)
                else:
                    name = os.path.basename(
                        frame_filename("", utcnow(), n))
                    with open(os.path.join(args.out, name), "wb") as fh:
                        fh.write(raw)
                    state["saved"] += 1
                    entry.update({"file": name})
                    print("wrote %s (%d bytes)" % (name, len(raw)), flush=True)
            elif kind == "CompressedImage":
                raw = bytes(bytearray(sample.data))
                entry.update({
                    "stamp_sec": sample.header.stamp.sec,
                    "stamp_nsec": sample.header.stamp.nanosec,
                    "frame_id": sample.header.frame_id,
                    "bytes": len(raw),
                    "jpeg": is_jpeg(raw),
                })
                if not is_jpeg(raw):
                    state["dropped"] += 1
                    entry["dropped"] = True
                    print("dropped non-JPEG frame on %s (%d bytes)"
                          % (ros_topic, len(raw)), flush=True)
                else:
                    name = os.path.basename(
                        frame_filename("", utcnow(), n))
                    with open(os.path.join(args.out, name), "wb") as fh:
                        fh.write(raw)
                    state["saved"] += 1
                    entry.update({"file": name})
                    print("wrote %s (%d bytes)" % (name, len(raw)), flush=True)
            else:
                state["other"] += 1
                entry["repr"] = repr(sample)[:2000]
            index_fh.write(json.dumps(entry) + "\n")
            index_fh.flush()
        except Exception as exc:
            state["dropped"] += 1
            print("dropped sample on %s: %s" % (ros_topic, exc), flush=True)

    try:
        return _stats_loop(readers, _rec, args)
    finally:
        index_fh.close()
        print("index: %s (saved %d frames, dropped %d, other %d)"
              % (index_path, state["saved"], state["dropped"], state["other"]))


# --- CLI ------------------------------------------------------------------

def build_parser():
    p = argparse.ArgumentParser(
        description="Capture XRCE-DDS data from the ESP32-S3 (no ROS 2 needed).")
    # Topic selection lives here so it works AFTER the subcommand, e.g.
    # `capture.py save --out ./frames --topic /cam=CompressedImage`.
    sel = argparse.ArgumentParser(add_help=False)
    sel.add_argument("--domain", type=int, default=0,
                     help="DDS domain id (match S3 domain_id, default 0)")
    sel.add_argument("--topic", action="append", default=[],
                     help="ROS topic, optionally NAME=Type (repeatable). "
                          "Default: /camera/image/compressed")
    sub = p.add_subparsers(dest="cmd", required=True)

    a = sub.add_parser("agent", help="run MicroXRCEAgent (UDP)")
    a.add_argument("--port", type=int, default=8888)
    a.add_argument("--agent-bin", default=None)
    a.add_argument("agent_args", nargs="*",
                   help="extra args passed to the agent")
    a.set_defaults(func=cmd_agent)

    d = sub.add_parser("discover", parents=[sel],
                       help="report which topics actually deliver samples")
    d.add_argument("--wait", type=float, default=5.0)
    d.set_defaults(func=cmd_discover)

    for name, func, help_ in (
            ("echo", cmd_echo, "print per-message stats"),
            ("save", cmd_save, "write JPEG frames to --out"),
            ("record", cmd_record, "store all selected topics + index.jsonl")):
        s = sub.add_parser(name, parents=[sel], help=help_)
        if name in ("save", "record"):
            s.add_argument("--out", required=True)
        s.add_argument("--stats-every", type=float, default=None,
                       help="stats line interval in s (0 disables)")
        s.set_defaults(func=func)
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    try:
        return args.func(args) or 0
    except ValueError as exc:
        print("error: %s" % exc, file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
