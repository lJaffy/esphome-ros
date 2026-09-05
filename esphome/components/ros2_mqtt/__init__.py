import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID
from esphome.types import ConfigType

DEPENDENCIES = ["mqtt", "ros2"]
AUTO_LOAD = ["json"]

ros2_mqtt_ns = cg.esphome_ns.namespace("ros2_mqtt")
Ros2MqttComponent = ros2_mqtt_ns.class_("Ros2MqttComponent", cg.Component)

CONF_TOPIC_PREFIX = "topic_prefix"
CONF_DEFAULT_QOS = "default_qos"
CONF_DEFAULT_RETAIN = "default_retain"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(Ros2MqttComponent),
        cv.Optional(CONF_TOPIC_PREFIX, default=""): cv.string,
        cv.Optional(CONF_DEFAULT_QOS, default=0): cv.mqtt_qos,
        cv.Optional(CONF_DEFAULT_RETAIN, default=False): cv.boolean,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config: ConfigType) -> None:
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_topic_prefix(config[CONF_TOPIC_PREFIX]))
    cg.add(var.set_default_qos(config[CONF_DEFAULT_QOS]))
    cg.add(var.set_default_retain(config[CONF_DEFAULT_RETAIN]))
