import logging
import math
import re
from collections.abc import Iterator

import esphome.codegen as cg
from esphome import final_validate as fv
from esphome.components import binary_sensor as bs_comp
from esphome.components.binary_sensor import BinarySensor
from esphome.components.esp32_camera import ESP32Camera
from esphome.components.light import LightState
from esphome.components.sensor import Sensor
from esphome.components.servo import Servo
from esphome.components.switch import Switch
from esphome.components import time as time_comp
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_INTERVAL,
    CONF_TIME_ID,
    CONF_TOPIC,
    CONF_TYPE,
    CONF_UPDATE_INTERVAL,
)
from esphome.types import ConfigType

_LOGGER = logging.getLogger(__name__)


def _auto_load(config=None):
    # Config-conditional: only pull the entity libs the YAML actually uses.
    # Called with no args during dependency resolution -> full superset.
    # Codec (ros2_json.cpp) and type table stay whole: no per-type guards.
    full = ["json", "binary_sensor", "sensor", "switch", "servo", "camera", "light"]
    if not config:
        return full
    libs = {"json"}
    for sub in config.get(CONF_SUBSCRIPTIONS, []):
        if (target := sub.get(CONF_TARGET)) is not None:
            for kind in ("servo", "switch", "light"):
                if target.get(kind) is not None:
                    libs.add(kind)
            if target.get(CONF_DIFF_DRIVE) is not None:
                libs.add("servo")
        elif sub.get(CONF_TARGETS) is not None:
            libs.add("servo")
    for pub in config.get(CONF_PUBLICATIONS, []):
        if (source := pub.get(CONF_SOURCE)) is not None:
            for kind in ("sensor", "switch", "binary_sensor", "camera", "light"):
                if source.get(kind) is not None:
                    libs.add(kind)
            if source.get(CONF_IMU) is not None:
                libs.add("sensor")
            if source.get(CONF_GPS) is not None:
                libs.add("sensor")
        elif pub.get(CONF_SOURCES) is not None:
            libs.add("servo")
    if config.get(CONF_STATUS_SENSOR) is not None:
        libs.add("binary_sensor")
    return sorted(libs, key=full.index)


AUTO_LOAD = _auto_load

ros2_ns = cg.esphome_ns.namespace("ros2")
Ros2Component = ros2_ns.class_("Ros2Component", cg.Component)

CONF_MIDDLEWARE = "middleware"
CONF_DEFAULT_PUBLISH_INTERVAL = "default_publish_interval"
CONF_SUBSCRIPTIONS = "subscriptions"
CONF_PUBLICATIONS = "publications"
CONF_TARGET = "target"
CONF_TARGETS = "targets"
CONF_SOURCE = "source"
CONF_SOURCES = "sources"
CONF_STATUS_SENSOR = "status_sensor"
CONF_JOINT_NAME = "joint_name"
CONF_MIN_RAD = "min_rad"
CONF_MAX_RAD = "max_rad"
CONF_FIELD = "field"
CONF_QOS = "qos"
CONF_FRAME_ID = "frame_id"
CONF_RADIATION_TYPE = "radiation_type"
CONF_FIELD_OF_VIEW = "field_of_view"
CONF_MIN_RANGE = "min_range"
CONF_MAX_RANGE = "max_range"
CONF_VARIANCE = "variance"
CONF_MIN_VOLTAGE = "min_voltage"
CONF_MAX_VOLTAGE = "max_voltage"
CONF_DESIGN_CAPACITY = "design_capacity"
CONF_TECHNOLOGY = "technology"
CONF_LOCATION = "location"
CONF_DIFF_DRIVE = "diff_drive"
CONF_LEFT = "left"
CONF_RIGHT = "right"
CONF_WHEEL_SEPARATION = "wheel_separation"
CONF_MAX_LINEAR_SPEED = "max_linear_speed"
CONF_MAX_ANGULAR_SPEED = "max_angular_speed"
CONF_CMD_TIMEOUT = "cmd_timeout"
CONF_ODOM = "odom"
CONF_TRANSFORMS = "transforms"
CONF_CHILD_FRAME_ID = "child_frame_id"
CONF_TF_TOPIC = "tf_topic"
CONF_TRANSLATION = "translation"
CONF_ROTATION = "rotation"
CONF_IMU = "imu"
CONF_GPS = "gps"
CONF_LATITUDE = "latitude"
CONF_LONGITUDE = "longitude"
CONF_ALTITUDE = "altitude"
CONF_ACCEL_X = "accel_x"
CONF_ACCEL_Y = "accel_y"
CONF_ACCEL_Z = "accel_z"
CONF_GYRO_X = "gyro_x"
CONF_GYRO_Y = "gyro_y"
CONF_GYRO_Z = "gyro_z"
CONF_ORIENTATION_X = "orientation_x"
CONF_ORIENTATION_Y = "orientation_y"
CONF_ORIENTATION_Z = "orientation_z"
CONF_ORIENTATION_W = "orientation_w"
CONF_RAW = "raw"

SCALAR_TYPES = [
    "std_msgs/Bool",
    "std_msgs/Float32",
    "std_msgs/Int32",
    "std_msgs/String",
]
MULTI_JOINT_TYPES = [
    "sensor_msgs/JointState",
    "trajectory_msgs/JointTrajectory",
]
LIGHT_TYPES = [
    "std_msgs/ColorRGBA",
    "sensor_msgs/Joy",
]
IMAGE_TYPES = [
    "sensor_msgs/CompressedImage",
]
TELEMETRY_TYPES = [
    "sensor_msgs/Range",
    "sensor_msgs/BatteryState",
]
MOTION_TYPES = [
    "geometry_msgs/Twist",
    "nav_msgs/Odometry",
    "tf2_msgs/TFMessage",
]
IMU_TYPES = [
    "sensor_msgs/Imu",
]
GPS_TYPES = [
    "sensor_msgs/NavSatFix",
]
SUPPORTED_TYPES = SCALAR_TYPES + MULTI_JOINT_TYPES + LIGHT_TYPES + IMAGE_TYPES + TELEMETRY_TYPES + MOTION_TYPES + IMU_TYPES + GPS_TYPES

# Types with std_msgs/Header: stamp comes from the time: source, frame_id
# from each publication's frame_id:.
HEADER_TYPES = [
    "sensor_msgs/JointState",
    "trajectory_msgs/JointTrajectory",
    "sensor_msgs/Joy",
    "sensor_msgs/CompressedImage",
    "sensor_msgs/Range",
    "sensor_msgs/BatteryState",
    "nav_msgs/Odometry",
    "sensor_msgs/Imu",
    "sensor_msgs/NavSatFix",
]

QOS_LEVELS = ["reliable", "best_effort"]

RADIATION_TYPES = {"ultrasound": 0, "infrared": 1}

BATTERY_TECHNOLOGIES = {
    "unknown": 0,
    "nimh": 1,
    "lion": 2,
    "lipo": 3,
    "life": 4,
    "nicd": 5,
    "limn": 6,
    "ternary": 7,
    "vrla": 8,
}

RANGE_PARAMS = (CONF_RADIATION_TYPE, CONF_FIELD_OF_VIEW, CONF_MIN_RANGE, CONF_MAX_RANGE, CONF_VARIANCE)
BATTERY_PARAMS = (
    CONF_MIN_VOLTAGE, CONF_MAX_VOLTAGE, CONF_DESIGN_CAPACITY, CONF_TECHNOLOGY, CONF_LOCATION,
)

# Codec-complete (JSON + XCDR + parity) but with no entity mapping yet:
# dispatch and poll silently ignore these, so validation rejects them loudly
# until a target:/source: binding exists.
UNMAPPED_TYPES = [
    "std_msgs/Int32",
    "std_msgs/String",
]

LIGHT_FIELDS = ["rgb", "brightness"]

_FRAME_ID_RE = re.compile(r"^[A-Za-z0-9/_-]*$")


def _frame_id(value):
    value = cv.string(value)
    if not _FRAME_ID_RE.match(value):
        raise cv.Invalid(
            "frame_id: may only contain A-Za-z0-9/_- "
            "(the hand-encoded image JSON has no string escaper)")
    if len(value) >= 64:
        raise cv.Invalid("frame_id: must fit in 63 chars + NUL")
    return value


def _entity_ref(entity_cls):
    return cv.Schema(
        {
            cv.Required(CONF_ID): cv.use_id(entity_cls),
            cv.Optional(CONF_FIELD): cv.string,
        }
    )


def _servo_target_schema():
    return cv.Schema(
        {
            cv.Required(CONF_ID): cv.use_id(Servo),
            cv.Optional(CONF_FIELD, default="position"): cv.string,
            cv.Optional(CONF_JOINT_NAME): cv.string,
            cv.Optional(CONF_MIN_RAD, default=-math.pi): cv.float_,
            cv.Optional(CONF_MAX_RAD, default=math.pi): cv.float_,
        }
    )


def _light_target_schema():
    return cv.Schema(
        {
            cv.Required(CONF_ID): cv.use_id(LightState),
            cv.Optional(CONF_FIELD, default="rgb"): cv.one_of(*LIGHT_FIELDS),
        }
    )


def _wheel_schema():
    return cv.Schema(
        {
            cv.Required(CONF_ID): cv.use_id(Servo),
        }
    )


def _diff_drive_schema():
    return cv.Schema(
        {
            cv.Required(CONF_LEFT): _wheel_schema(),
            cv.Required(CONF_RIGHT): _wheel_schema(),
            cv.Required(CONF_WHEEL_SEPARATION): cv.positive_float,
            cv.Optional(CONF_MAX_LINEAR_SPEED, default=0.5): cv.positive_float,
            cv.Optional(CONF_MAX_ANGULAR_SPEED, default=2.0): cv.positive_float,
            cv.Optional(CONF_CMD_TIMEOUT, default="500ms"): cv.positive_time_period_milliseconds,
        }
    )


def _single_target_schema():
    return cv.Schema(
        {
            cv.Optional("servo"): _servo_target_schema(),
            cv.Optional("switch"): _entity_ref(Switch),
            cv.Optional("light"): _light_target_schema(),
            cv.Optional(CONF_DIFF_DRIVE): _diff_drive_schema(),
        }
    )


def _camera_source_schema() -> cv.Schema:
    # NOTE: esp32_camera::ESP32Camera does NOT declare camera::Camera as a
    # Python codegen parent (it lists PollingComponent + EntityBase), so
    # cv.use_id(camera::Camera) always rejects esp32_camera IDs with
    # "doesn't inherit from camera::Camera". Validate against the concrete
    # ESP32Camera class instead; the C++ upcast to camera::Camera* is safe
    # (ESP32Camera final : public camera::Camera in esp32_camera.h).
    return cv.Schema(
        {
            cv.Required(CONF_ID): cv.use_id(ESP32Camera),
        }
    )


def _odom_source_schema():
    return cv.Schema(
        {
            cv.Required(CONF_WHEEL_SEPARATION): cv.positive_float,
        }
    )


def _imu_source_schema():
    return cv.Schema(
        {
            cv.Required(CONF_ACCEL_X): _entity_ref(Sensor),
            cv.Required(CONF_ACCEL_Y): _entity_ref(Sensor),
            cv.Required(CONF_ACCEL_Z): _entity_ref(Sensor),
            cv.Required(CONF_GYRO_X): _entity_ref(Sensor),
            cv.Required(CONF_GYRO_Y): _entity_ref(Sensor),
            cv.Required(CONF_GYRO_Z): _entity_ref(Sensor),
            cv.Optional(CONF_ORIENTATION_X): _entity_ref(Sensor),
            cv.Optional(CONF_ORIENTATION_Y): _entity_ref(Sensor),
            cv.Optional(CONF_ORIENTATION_Z): _entity_ref(Sensor),
            cv.Optional(CONF_ORIENTATION_W): _entity_ref(Sensor),
        }
    )


def _gps_source_schema():
    return cv.Schema(
        {
            cv.Required(CONF_LATITUDE): _entity_ref(Sensor),
            cv.Required(CONF_LONGITUDE): _entity_ref(Sensor),
            cv.Optional(CONF_ALTITUDE): _entity_ref(Sensor),
        }
    )


def _single_source_schema():
    return cv.Schema(
        {
            cv.Optional("sensor"): _entity_ref(Sensor),
            cv.Optional("switch"): _entity_ref(Switch),
            cv.Optional("binary_sensor"): _entity_ref(BinarySensor),
            cv.Optional("camera"): _camera_source_schema(),
            cv.Optional("light"): _light_target_schema(),
            cv.Optional(CONF_ODOM): _odom_source_schema(),
            cv.Optional(CONF_IMU): _imu_source_schema(),
            cv.Optional(CONF_GPS): _gps_source_schema(),
        }
    )


def _transform_schema():
    return cv.Schema(
        {
            cv.Required(CONF_FRAME_ID): _frame_id,
            cv.Required(CONF_CHILD_FRAME_ID): _frame_id,
            cv.Required(CONF_TRANSLATION): cv.All(
                cv.ensure_list(cv.float_), cv.Length(min=3, max=3)),
            cv.Required(CONF_ROTATION): cv.All(
                cv.ensure_list(cv.float_), cv.Length(min=4, max=4)),
        }
    )


def _validate_subscription(config: ConfigType) -> ConfigType:
    has_target = CONF_TARGET in config
    has_targets = CONF_TARGETS in config
    if has_target == has_targets:
        raise cv.Invalid(
            "Use exactly one of target: or targets: per subscription")
    type_ = config[CONF_TYPE]
    if type_ in UNMAPPED_TYPES:
        raise cv.Invalid(
            f"Type {type_} is schema-reserved: codec exists but no "
            "target: entity mapping yet")
    if type_ in TELEMETRY_TYPES:
        raise cv.Invalid(
            f"Type {type_} is publish-only (no target: entity consumes it)")
    if type_ in IMU_TYPES:
        raise cv.Invalid(
            f"Type {type_} is publish-only (no target: entity consumes it)")
    if type_ in GPS_TYPES:
        raise cv.Invalid(
            f"Type {type_} is publish-only (no target: entity consumes it)")
    if type_ in MULTI_JOINT_TYPES and not has_targets:
        raise cv.Invalid(f"Type {type_} requires targets: (plural)")
    if type_ not in MULTI_JOINT_TYPES and has_targets:
        raise cv.Invalid(f"Type {type_} requires target: (singular)")
    if has_target:
        kinds = [k for k in ("servo", "switch", "light", CONF_DIFF_DRIVE)
                 if config[CONF_TARGET].get(k) is not None]
        if len(kinds) != 1:
            raise cv.Invalid(
                "target: needs exactly one of servo:, switch:, light:, diff_drive:")
        if "servo" in kinds and type_ == "std_msgs/Bool":
            raise cv.Invalid("servo: targets need std_msgs/Float32")
        if "servo" in kinds and type_ in LIGHT_TYPES:
            raise cv.Invalid(f"servo: targets cannot use {type_} (needs light:)")
        if "switch" in kinds and type_ != "std_msgs/Bool":
            raise cv.Invalid("switch: targets need std_msgs/Bool")
        if "light" in kinds and type_ not in LIGHT_TYPES:
            raise cv.Invalid(f"light: targets need one of {LIGHT_TYPES}")
        if CONF_DIFF_DRIVE in kinds and type_ != "geometry_msgs/Twist":
            raise cv.Invalid(
                f"diff_drive: targets need geometry_msgs/Twist (got {type_})")
        if type_ == "geometry_msgs/Twist" and CONF_DIFF_DRIVE not in kinds:
            raise cv.Invalid(
                "geometry_msgs/Twist needs a diff_drive: target (left:/right: wheel servos)")
    if has_targets:
        seen = set()
        for entry in config[CONF_TARGETS]:
            servo = entry.get("servo")
            if servo is None:
                raise cv.Invalid(
                    "Multi-joint targets only support servo: entries")
            name = servo.get(CONF_JOINT_NAME)
            if not name:
                raise cv.Invalid(
                    "joint_name is required for multi-joint servo targets")
            if name in seen:
                raise cv.Invalid(f"Duplicate joint_name '{name}'")
            seen.add(name)
    return config


SUBSCRIPTION_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.Required(CONF_TOPIC): cv.string,
            cv.Required(CONF_TYPE): cv.one_of(*SUPPORTED_TYPES),
            cv.Optional(CONF_TARGET): _single_target_schema(),
            cv.Optional(CONF_TARGETS): cv.ensure_list(
                cv.Schema({cv.Required("servo"): _servo_target_schema()})
            ),
            cv.Optional(CONF_QOS): cv.one_of(*QOS_LEVELS),
        }
    ),
    _validate_subscription,
)


def _validate_publication(config: ConfigType) -> ConfigType:
    type_ = config[CONF_TYPE]
    if type_ in UNMAPPED_TYPES:
        raise cv.Invalid(
            f"Type {type_} is schema-reserved: codec exists but no "
            "source: entity mapping yet")
    is_tf = type_ == "tf2_msgs/TFMessage"
    has_source = CONF_SOURCE in config
    has_sources = CONF_SOURCES in config
    if is_tf:
        if has_source or has_sources:
            raise cv.Invalid(
                "tf2_msgs/TFMessage uses transforms:, not source:/sources:")
        transforms = config.get(CONF_TRANSFORMS, [])
        if not 1 <= len(transforms) <= 4:
            raise cv.Invalid(
                "tf2_msgs/TFMessage needs 1-4 transforms: entries")
    else:
        if has_source == has_sources:
            raise cv.Invalid(
                "Use exactly one of source: or sources: per publication")
        if type_ in MULTI_JOINT_TYPES and not has_sources:
            raise cv.Invalid(f"Type {type_} requires sources: (plural)")
        if type_ not in MULTI_JOINT_TYPES and has_sources:
            raise cv.Invalid(f"Type {type_} requires source: (singular)")
    if has_source:
        kinds = [k for k in ("sensor", "switch", "binary_sensor", "camera", "light", CONF_ODOM, CONF_IMU,
                             CONF_GPS)
                 if config[CONF_SOURCE].get(k) is not None]
        if len(kinds) != 1:
            raise cv.Invalid(
                "source: needs exactly one of sensor:, switch:, binary_sensor:, camera:, light:, odom:, imu:, "
                "gps:")
        if "sensor" in kinds and type_ not in ("std_msgs/Float32", *TELEMETRY_TYPES):
            raise cv.Invalid(
                "sensor: sources need std_msgs/Float32, sensor_msgs/Range, "
                "or sensor_msgs/BatteryState")
        if type_ in TELEMETRY_TYPES and "sensor" not in kinds:
            raise cv.Invalid(
                f"Type {type_} needs a sensor: source (single distance/voltage sensor)")
        if ("switch" in kinds or "binary_sensor" in kinds) and type_ != "std_msgs/Bool":
            raise cv.Invalid(
                "switch:/binary_sensor: sources need std_msgs/Bool")
        if "camera" in kinds and type_ not in IMAGE_TYPES:
            raise cv.Invalid("camera: sources need sensor_msgs/CompressedImage")
        if "light" in kinds and type_ != "std_msgs/ColorRGBA":
            raise cv.Invalid("light: sources need std_msgs/ColorRGBA")
        if CONF_ODOM in kinds and type_ != "nav_msgs/Odometry":
            raise cv.Invalid(
                f"odom: sources need nav_msgs/Odometry (got {type_})")
        if type_ == "nav_msgs/Odometry" and CONF_ODOM not in kinds:
            raise cv.Invalid(
                "nav_msgs/Odometry needs an odom: source (wheel separation)")
        if CONF_IMU in kinds and type_ != "sensor_msgs/Imu":
            raise cv.Invalid(
                f"imu: sources need sensor_msgs/Imu (got {type_})")
        if type_ == "sensor_msgs/Imu" and CONF_IMU not in kinds:
            raise cv.Invalid(
                "sensor_msgs/Imu needs an imu: source "
                "(accel_x/y/z + gyro_x/y/z, optional orientation_x/y/z/w)")
        if CONF_GPS in kinds and type_ != "sensor_msgs/NavSatFix":
            raise cv.Invalid(
                f"gps: sources need sensor_msgs/NavSatFix (got {type_})")
        if type_ == "sensor_msgs/NavSatFix" and CONF_GPS not in kinds:
            raise cv.Invalid(
                "sensor_msgs/NavSatFix needs a gps: source "
                "(latitude + longitude, optional altitude)")
    if type_ == "sensor_msgs/Imu" and has_source:
        imu = config[CONF_SOURCE].get(CONF_IMU, {})
        orientation_keys = [CONF_ORIENTATION_X, CONF_ORIENTATION_Y,
                            CONF_ORIENTATION_Z, CONF_ORIENTATION_W]
        present = [k for k in orientation_keys if imu.get(k) is not None]
        if present and len(present) != 4:
            raise cv.Invalid(
                "imu: orientation needs all of orientation_x/y/z/w or none")
    if type_ == "sensor_msgs/Joy" and has_source:
        raise cv.Invalid("sensor_msgs/Joy is subscribe-only (no source entity produces axes/buttons)")
    if CONF_FRAME_ID in config and type_ not in HEADER_TYPES:
        raise cv.Invalid(
            f"Type {type_} has no header; frame_id: needs one of {HEADER_TYPES}")
    for key in RANGE_PARAMS:
        if key in config and type_ != "sensor_msgs/Range":
            raise cv.Invalid(f"{key}: only valid with sensor_msgs/Range")
    for key in BATTERY_PARAMS:
        if key in config and type_ != "sensor_msgs/BatteryState":
            raise cv.Invalid(f"{key}: only valid with sensor_msgs/BatteryState")
    if CONF_TRANSFORMS in config and not is_tf:
        raise cv.Invalid("transforms: only valid with tf2_msgs/TFMessage")
    if CONF_TF_TOPIC in config and type_ != "nav_msgs/Odometry":
        raise cv.Invalid("tf_topic: only valid with nav_msgs/Odometry")
    if CONF_CHILD_FRAME_ID in config and type_ != "nav_msgs/Odometry":
        raise cv.Invalid("child_frame_id: only valid with nav_msgs/Odometry")
    if CONF_MIN_RANGE in config or CONF_MAX_RANGE in config:
        if config.get(CONF_MIN_RANGE, 0.0) > config.get(CONF_MAX_RANGE, 0.0):
            raise cv.Invalid("min_range: must not exceed max_range:")
    if CONF_MIN_VOLTAGE in config or CONF_MAX_VOLTAGE in config:
        if config.get(CONF_MIN_VOLTAGE, 0.0) >= config.get(CONF_MAX_VOLTAGE, 0.0):
            raise cv.Invalid("min_voltage: must be below max_voltage:")
    if config.get(CONF_RAW, False):
        sensor_backed = ("std_msgs/Float32", *TELEMETRY_TYPES, *IMU_TYPES, *GPS_TYPES)
        if type_ not in sensor_backed:
            raise cv.Invalid(f"{CONF_RAW}: only valid with sensor-backed types {list(sensor_backed)}")
    return config


PUBLICATION_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.Required(CONF_TOPIC): cv.string,
            cv.Required(CONF_TYPE): cv.one_of(*SUPPORTED_TYPES),
            cv.Optional(CONF_SOURCE): _single_source_schema(),
            cv.Optional(CONF_SOURCES): cv.ensure_list(
                cv.Schema({cv.Required("servo"): _servo_target_schema()})
            ),
            cv.Optional(CONF_INTERVAL): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_RAW, default=False): cv.boolean,
            cv.Optional(CONF_FRAME_ID): _frame_id,
            cv.Optional(CONF_QOS): cv.one_of(*QOS_LEVELS),
            cv.Optional(CONF_CHILD_FRAME_ID): _frame_id,
            cv.Optional(CONF_TF_TOPIC): cv.string,
            cv.Optional(CONF_TRANSFORMS): cv.ensure_list(_transform_schema()),
            cv.Optional(CONF_RADIATION_TYPE): cv.one_of(*RADIATION_TYPES),
            cv.Optional(CONF_FIELD_OF_VIEW): cv.float_,
            cv.Optional(CONF_MIN_RANGE): cv.float_,
            cv.Optional(CONF_MAX_RANGE): cv.float_,
            cv.Optional(CONF_VARIANCE): cv.float_,
            cv.Optional(CONF_MIN_VOLTAGE): cv.float_,
            cv.Optional(CONF_MAX_VOLTAGE): cv.float_,
            cv.Optional(CONF_DESIGN_CAPACITY): cv.float_,
            cv.Optional(CONF_TECHNOLOGY): cv.one_of(*BATTERY_TECHNOLOGIES),
            cv.Optional(CONF_LOCATION): cv.string,
        }
    ),
    _validate_publication,
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(Ros2Component),
        cv.Required(CONF_MIDDLEWARE): cv.string,
        cv.Optional(CONF_DEFAULT_PUBLISH_INTERVAL, default="1s"): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_SUBSCRIPTIONS, default=[]): cv.ensure_list(SUBSCRIPTION_SCHEMA),
        cv.Optional(CONF_PUBLICATIONS, default=[]): cv.ensure_list(PUBLICATION_SCHEMA),
        cv.Optional(CONF_STATUS_SENSOR): bs_comp.binary_sensor_schema(BinarySensor),
        cv.Optional(CONF_TIME_ID): cv.use_id(time_comp.RealTimeClock),
    }
).extend(cv.COMPONENT_SCHEMA)


_IMU_SENSOR_KEYS = (
    CONF_ACCEL_X, CONF_ACCEL_Y, CONF_ACCEL_Z,
    CONF_GYRO_X, CONF_GYRO_Y, CONF_GYRO_Z,
    CONF_ORIENTATION_X, CONF_ORIENTATION_Y,
    CONF_ORIENTATION_Z, CONF_ORIENTATION_W,
)
_GPS_SENSOR_KEYS = (CONF_LATITUDE, CONF_LONGITUDE, CONF_ALTITUDE)


def _iter_publication_sensor_ids(pub: ConfigType) -> Iterator[str]:
    source = pub.get(CONF_SOURCE)
    if not source:
        return
    if (ref := source.get("sensor")) is not None:
        yield ref[CONF_ID]
    if (imu := source.get(CONF_IMU)) is not None:
        for key in _IMU_SENSOR_KEYS:
            if (ref := imu.get(key)) is not None:
                yield ref[CONF_ID]
    if (gps := source.get(CONF_GPS)) is not None:
        for key in _GPS_SENSOR_KEYS:
            if (ref := gps.get(key)) is not None:
                yield ref[CONF_ID]


def _final_validate(config: ConfigType) -> None:
    pubs = config.get(CONF_PUBLICATIONS, [])
    if not pubs:
        return
    try:
        fconf = fv.full_config.get()
    except LookupError:
        return
    default_interval = config.get(CONF_DEFAULT_PUBLISH_INTERVAL)
    for pub in pubs:
        if pub.get(CONF_TYPE) not in ("std_msgs/Float32", *TELEMETRY_TYPES, *IMU_TYPES, *GPS_TYPES):
            continue
        interval = pub.get(CONF_INTERVAL, default_interval)
        if interval is None:
            continue
        for sensor_id in _iter_publication_sensor_ids(pub):
            try:
                sensor_path = fconf.get_path_for_id(sensor_id)[:-1]
                sensor_config = fconf.get_config_for_path(sensor_path)
            except KeyError:
                continue
            update_interval = sensor_config.get(CONF_UPDATE_INTERVAL)
            if update_interval is None or interval >= update_interval:
                continue
            _LOGGER.warning(
                "ros2 publication '%s' publishes every %s but sensor '%s' only "
                "updates every %s: ROS will republish stale values. Set the "
                "sensor's update_interval to %s or faster, and add "
                "'filters: [{throttle: %s}]' to keep Home Assistant traffic "
                "unchanged (pairs with 'raw: true').",
                pub.get(CONF_TOPIC), interval, sensor_id, update_interval,
                interval, update_interval,
            )


FINAL_VALIDATE_SCHEMA = _final_validate


async def to_code(config: ConfigType) -> None:
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_middleware_name(config[CONF_MIDDLEWARE]))
    cg.add(var.set_default_publish_interval(
        config[CONF_DEFAULT_PUBLISH_INTERVAL]))
    # The servo component never defines USE_SERVO (no IS_PLATFORM_COMPONENT,
    # no in-tree optional users), so declare it here whenever this bridge
    # emits servo calls. Every other USE_* we test is defined by its own
    # component's to_code (e.g. sensor) exactly when that entity is built.
    uses_servo = any(
        sub.get(CONF_TARGET, {}).get("servo") is not None
        or CONF_DIFF_DRIVE in sub.get(CONF_TARGET, {})
        or CONF_TARGETS in sub
        for sub in config[CONF_SUBSCRIPTIONS]
    ) or any(CONF_SOURCES in pub for pub in config[CONF_PUBLICATIONS])
    if uses_servo:
        cg.add_define("USE_SERVO")
    if (time_id := config.get(CONF_TIME_ID)) is not None:
        time_var = await cg.get_variable(time_id)
        cg.add(var.set_time(time_var))

    for sub in config[CONF_SUBSCRIPTIONS]:
        topic = sub[CONF_TOPIC]
        type_ = sub[CONF_TYPE]
        if (target := sub.get(CONF_TARGET)) is not None:
            if (servo := target.get("servo")) is not None:
                ent = await cg.get_variable(servo[CONF_ID])
                cg.add(var.add_servo_subscription(topic, type_, ent,
                       servo[CONF_MIN_RAD], servo[CONF_MAX_RAD]))
            elif (sw := target.get("switch")) is not None:
                ent = await cg.get_variable(sw[CONF_ID])
                cg.add(var.add_switch_subscription(topic, type_, ent))
            elif (light := target.get("light")) is not None:
                ent = await cg.get_variable(light[CONF_ID])
                cg.add(var.add_light_subscription(topic, type_, ent, light[CONF_FIELD]))
            elif (dd := target.get(CONF_DIFF_DRIVE)) is not None:
                left = await cg.get_variable(dd[CONF_LEFT][CONF_ID])
                right = await cg.get_variable(dd[CONF_RIGHT][CONF_ID])
                cg.add(var.add_diff_drive_subscription(
                    topic, left, right,
                    dd[CONF_WHEEL_SEPARATION],
                    dd[CONF_MAX_LINEAR_SPEED],
                    dd[CONF_MAX_ANGULAR_SPEED],
                    dd[CONF_CMD_TIMEOUT],
                ))
        else:
            for entry in sub[CONF_TARGETS]:
                servo = entry["servo"]
                ent = await cg.get_variable(servo[CONF_ID])
                cg.add(
                    var.add_joint_subscription(
                        topic, type_, ent, servo[CONF_JOINT_NAME], servo[CONF_MIN_RAD], servo[CONF_MAX_RAD]
                    )
                )
        if (qos := sub.get(CONF_QOS)) is not None:
            cg.add(var.set_subscription_qos(topic, qos))

    for pub in config[CONF_PUBLICATIONS]:
        topic = pub[CONF_TOPIC]
        type_ = pub[CONF_TYPE]
        interval = pub.get(
            CONF_INTERVAL, config[CONF_DEFAULT_PUBLISH_INTERVAL])
        if (source := pub.get(CONF_SOURCE)) is not None:
            if (sens := source.get("sensor")) is not None:
                ent = await cg.get_variable(sens[CONF_ID])
                if type_ == "sensor_msgs/Range":
                    cg.add(var.add_range_publication(topic, ent, interval))
                elif type_ == "sensor_msgs/BatteryState":
                    cg.add(var.add_battery_publication(topic, ent, interval))
                else:
                    cg.add(var.add_sensor_publication(topic, type_, ent, interval))
            elif (sw := source.get("switch")) is not None:
                ent = await cg.get_variable(sw[CONF_ID])
                cg.add(var.add_switch_publication(topic, type_, ent, interval))
            elif (bs := source.get("binary_sensor")) is not None:
                ent = await cg.get_variable(bs[CONF_ID])
                cg.add(var.add_binary_sensor_publication(
                    topic, type_, ent, interval))
            elif (cam := source.get("camera")) is not None:
                ent = await cg.get_variable(cam[CONF_ID])
                cg.add(var.add_image_publication(topic, type_, ent, interval))
            elif (light := source.get("light")) is not None:
                ent = await cg.get_variable(light[CONF_ID])
                cg.add(var.add_light_publication(topic, type_, ent, interval))
            elif (odom := source.get(CONF_ODOM)) is not None:
                cg.add(var.add_odom_publication(topic, interval))
                cg.add(var.set_odom_params(
                    topic,
                    odom[CONF_WHEEL_SEPARATION],
                    pub.get(CONF_CHILD_FRAME_ID, "base_link"),
                    pub.get(CONF_TF_TOPIC, ""),
                ))
            elif (imu := source.get(CONF_IMU)) is not None:
                accel_x = await cg.get_variable(imu[CONF_ACCEL_X][CONF_ID])
                accel_y = await cg.get_variable(imu[CONF_ACCEL_Y][CONF_ID])
                accel_z = await cg.get_variable(imu[CONF_ACCEL_Z][CONF_ID])
                gyro_x = await cg.get_variable(imu[CONF_GYRO_X][CONF_ID])
                gyro_y = await cg.get_variable(imu[CONF_GYRO_Y][CONF_ID])
                gyro_z = await cg.get_variable(imu[CONF_GYRO_Z][CONF_ID])
                cg.add(var.add_imu_publication(topic, interval))
                cg.add(var.set_imu_sources(
                    topic,
                    accel_x, accel_y, accel_z,
                    gyro_x, gyro_y, gyro_z,
                ))
                if imu.get(CONF_ORIENTATION_X) is not None:
                    ori_x = await cg.get_variable(imu[CONF_ORIENTATION_X][CONF_ID])
                    ori_y = await cg.get_variable(imu[CONF_ORIENTATION_Y][CONF_ID])
                    ori_z = await cg.get_variable(imu[CONF_ORIENTATION_Z][CONF_ID])
                    ori_w = await cg.get_variable(imu[CONF_ORIENTATION_W][CONF_ID])
                    cg.add(var.set_imu_orientation(
                        topic, ori_x, ori_y, ori_z, ori_w))
            elif (gps := source.get(CONF_GPS)) is not None:
                lat = await cg.get_variable(gps[CONF_LATITUDE][CONF_ID])
                lon = await cg.get_variable(gps[CONF_LONGITUDE][CONF_ID])
                cg.add(var.add_navsat_publication(topic, interval))
                cg.add(var.set_navsat_sources(topic, lat, lon))
                if gps.get(CONF_ALTITUDE) is not None:
                    alt = await cg.get_variable(gps[CONF_ALTITUDE][CONF_ID])
                    cg.add(var.set_navsat_altitude(topic, alt))
        elif type_ == "tf2_msgs/TFMessage":
            cg.add(var.add_tf_publication(topic, interval))
            for entry in pub.get(CONF_TRANSFORMS, []):
                t = entry[CONF_TRANSLATION]
                r = entry[CONF_ROTATION]
                cg.add(var.add_tf_transform(
                    topic,
                    entry[CONF_FRAME_ID], entry[CONF_CHILD_FRAME_ID],
                    t[0], t[1], t[2], r[0], r[1], r[2], r[3],
                ))
        else:
            cg.add(var.add_joint_state_publication(topic, type_, interval))
            for entry in pub[CONF_SOURCES]:
                servo = entry["servo"]
                ent = await cg.get_variable(servo[CONF_ID])
                cg.add(
                    var.add_joint_state_source(ent, servo[CONF_JOINT_NAME], servo[CONF_MIN_RAD],
                                               servo[CONF_MAX_RAD])
                )
        if (qos := pub.get(CONF_QOS)) is not None:
            cg.add(var.set_publication_qos(topic, qos))
        if pub.get(CONF_RAW, False):
            cg.add(var.set_publication_raw(topic, True))
        if (frame_id := pub.get(CONF_FRAME_ID)) is not None:
            cg.add(var.set_publication_frame_id(topic, frame_id))
        if type_ == "sensor_msgs/Range":
            cg.add(var.set_range_params(
                topic,
                RADIATION_TYPES[pub.get(CONF_RADIATION_TYPE, "ultrasound")],
                pub.get(CONF_FIELD_OF_VIEW, 0.5),
                pub.get(CONF_MIN_RANGE, 0.02),
                pub.get(CONF_MAX_RANGE, 4.0),
                pub.get(CONF_VARIANCE, 0.0),
            ))
        elif type_ == "sensor_msgs/BatteryState":
            cg.add(var.set_battery_params(
                topic,
                pub.get(CONF_MIN_VOLTAGE, 3.0),
                pub.get(CONF_MAX_VOLTAGE, 4.2),
                pub.get(CONF_DESIGN_CAPACITY, 0.0),
                BATTERY_TECHNOLOGIES[pub.get(CONF_TECHNOLOGY, "unknown")],
                pub.get(CONF_LOCATION, ""),
            ))

    if (status := config.get(CONF_STATUS_SENSOR)) is not None:
        sens = await bs_comp.new_binary_sensor(status)
        await cg.register_component(sens, status)
        cg.add(var.set_status_sensor(sens))
