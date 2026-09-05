import math

import esphome.codegen as cg
from esphome.components import binary_sensor as bs_comp
from esphome.components.binary_sensor import BinarySensor
from esphome.components.sensor import Sensor
from esphome.components.servo import Servo
from esphome.components.switch import Switch
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_INTERVAL, CONF_TOPIC, CONF_TYPE
from esphome.types import ConfigType


def _auto_load(config=None):
    # Servo lib must always be present: servo code paths are compiled
    # unconditionally so JointState dispatch links even for switch-only
    # configs. Configs without servos pay a small flash cost.
    return ["json", "binary_sensor", "sensor", "switch", "servo"]


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
SUPPORTED_TYPES = SCALAR_TYPES + MULTI_JOINT_TYPES


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


def _single_target_schema():
    return cv.Schema(
        {
            cv.Optional("servo"): _servo_target_schema(),
            cv.Optional("switch"): _entity_ref(Switch),
        }
    )


def _single_source_schema():
    return cv.Schema(
        {
            cv.Optional("sensor"): _entity_ref(Sensor),
            cv.Optional("switch"): _entity_ref(Switch),
            cv.Optional("binary_sensor"): _entity_ref(BinarySensor),
        }
    )


def _validate_subscription(config: ConfigType) -> ConfigType:
    has_target = CONF_TARGET in config
    has_targets = CONF_TARGETS in config
    if has_target == has_targets:
        raise cv.Invalid(
            "Use exactly one of target: or targets: per subscription")
    type_ = config[CONF_TYPE]
    if type_ in MULTI_JOINT_TYPES and not has_targets:
        raise cv.Invalid(f"Type {type_} requires targets: (plural)")
    if type_ not in MULTI_JOINT_TYPES and has_targets:
        raise cv.Invalid(f"Type {type_} requires target: (singular)")
    if has_target:
        kinds = [k for k in ("servo", "switch")
                 if config[CONF_TARGET].get(k) is not None]
        if len(kinds) != 1:
            raise cv.Invalid("target: needs exactly one of servo:, switch:")
        if "servo" in kinds and type_ == "std_msgs/Bool":
            raise cv.Invalid("servo: targets need std_msgs/Float32")
        if "switch" in kinds and type_ != "std_msgs/Bool":
            raise cv.Invalid("switch: targets need std_msgs/Bool")
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
        }
    ),
    _validate_subscription,
)


def _validate_publication(config: ConfigType) -> ConfigType:
    has_source = CONF_SOURCE in config
    has_sources = CONF_SOURCES in config
    if has_source == has_sources:
        raise cv.Invalid(
            "Use exactly one of source: or sources: per publication")
    type_ = config[CONF_TYPE]
    if type_ in MULTI_JOINT_TYPES and not has_sources:
        raise cv.Invalid(f"Type {type_} requires sources: (plural)")
    if type_ not in MULTI_JOINT_TYPES and has_sources:
        raise cv.Invalid(f"Type {type_} requires source: (singular)")
    if has_source:
        kinds = [k for k in ("sensor", "switch", "binary_sensor")
                 if config[CONF_SOURCE].get(k) is not None]
        if len(kinds) != 1:
            raise cv.Invalid(
                "source: needs exactly one of sensor:, switch:, binary_sensor:")
        if "sensor" in kinds and type_ != "std_msgs/Float32":
            raise cv.Invalid("sensor: sources need std_msgs/Float32")
        if ("switch" in kinds or "binary_sensor" in kinds) and type_ != "std_msgs/Bool":
            raise cv.Invalid(
                "switch:/binary_sensor: sources need std_msgs/Bool")
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
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config: ConfigType) -> None:
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_middleware_name(config[CONF_MIDDLEWARE]))
    cg.add(var.set_default_publish_interval(
        config[CONF_DEFAULT_PUBLISH_INTERVAL]))

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
        else:
            for entry in sub[CONF_TARGETS]:
                servo = entry["servo"]
                ent = await cg.get_variable(servo[CONF_ID])
                cg.add(
                    var.add_joint_subscription(
                        topic, type_, ent, servo[CONF_JOINT_NAME], servo[CONF_MIN_RAD], servo[CONF_MAX_RAD]
                    )
                )

    for pub in config[CONF_PUBLICATIONS]:
        topic = pub[CONF_TOPIC]
        type_ = pub[CONF_TYPE]
        interval = pub.get(
            CONF_INTERVAL, config[CONF_DEFAULT_PUBLISH_INTERVAL])
        if (source := pub.get(CONF_SOURCE)) is not None:
            if (sens := source.get("sensor")) is not None:
                ent = await cg.get_variable(sens[CONF_ID])
                cg.add(var.add_sensor_publication(topic, type_, ent, interval))
            elif (sw := source.get("switch")) is not None:
                ent = await cg.get_variable(sw[CONF_ID])
                cg.add(var.add_switch_publication(topic, type_, ent, interval))
            elif (bs := source.get("binary_sensor")) is not None:
                ent = await cg.get_variable(bs[CONF_ID])
                cg.add(var.add_binary_sensor_publication(
                    topic, type_, ent, interval))
        else:
            cg.add(var.add_joint_state_publication(topic, type_, interval))
            for entry in pub[CONF_SOURCES]:
                servo = entry["servo"]
                ent = await cg.get_variable(servo[CONF_ID])
                cg.add(
                    var.add_joint_state_source(ent, servo[CONF_JOINT_NAME], servo[CONF_MIN_RAD],
                                               servo[CONF_MAX_RAD])
                )

    if (status := config.get(CONF_STATUS_SENSOR)) is not None:
        sens = await bs_comp.new_binary_sensor(status)
        await cg.register_component(sens, status)
        cg.add(var.set_status_sensor(sens))
