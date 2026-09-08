from pathlib import Path

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID
from esphome.types import ConfigType

DEPENDENCIES = ["ros2", "network"]
CODEOWNERS = ["@anomalyco"]

CONF_AGENT_ADDRESS = "agent_address"
CONF_AGENT_PORT = "agent_port"
CONF_TRANSPORT = "transport"
CONF_TRANSPORT_TYPE = "type"
CONF_UART_ID = "uart_id"
CONF_DOMAIN_ID = "domain_id"
CONF_CLIENT_NAME = "client_name"
CONF_MAX_PACKET_LENGTH = "max_packet_length"
CONF_PROCESS_INTERVAL = "process_interval"
CONF_KEEPALIVE_TIMEOUT = "keepalive_timeout"
CONF_MAX_TOPICS = "max_topics"
CONF_MAX_DATAWRITERS = "max_datawriters"
CONF_MAX_DATAREADERS = "max_datareaders"

xrce_dds_ns = cg.esphome_ns.namespace("xrce_dds")
XrceDdsComponent = xrce_dds_ns.class_("XrceDdsComponent", cg.Component)

uart_ns = cg.esphome_ns.namespace("uart")
UARTComponent = uart_ns.class_("UARTComponent")


def _validate_transport(config: ConfigType) -> ConfigType:
    if config[CONF_TRANSPORT_TYPE] == "serial" and CONF_UART_ID not in config:
        raise cv.Invalid("transport: type serial requires uart_id")
    return config


TRANSPORT_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.Required(CONF_TRANSPORT_TYPE): cv.one_of("udp", "serial"),
            cv.Optional(CONF_UART_ID): cv.use_id(UARTComponent),
        }
    ),
    _validate_transport,
)


def _consume_socket(config: ConfigType) -> ConfigType:
    from esphome.components import socket

    socket.consume_sockets(1, "xrce_dds", socket.SocketType.UDP)(config)
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(XrceDdsComponent),
            cv.Required(CONF_AGENT_ADDRESS): cv.string,
            cv.Optional(CONF_AGENT_PORT, default=8888): cv.port,
            cv.Required(CONF_TRANSPORT): TRANSPORT_SCHEMA,
            cv.Optional(CONF_DOMAIN_ID, default=0): cv.int_range(min=0, max=255),
            cv.Optional(CONF_CLIENT_NAME, default="esp32-node"): cv.string,
            cv.Optional(CONF_MAX_PACKET_LENGTH, default=512): cv.positive_int,
            cv.Optional(CONF_PROCESS_INTERVAL, default="10ms"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_KEEPALIVE_TIMEOUT, default="5s"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_MAX_TOPICS, default=16): cv.int_range(min=1, max=16),
            cv.Optional(CONF_MAX_DATAWRITERS, default=8): cv.int_range(min=1, max=8),
            cv.Optional(CONF_MAX_DATAREADERS, default=8): cv.int_range(min=1, max=8),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    _consume_socket,
)


async def to_code(config: ConfigType) -> None:
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    third_party = Path(__file__).parent / "vendor"
    cg.add_build_flag(f"-I{third_party / 'microxrcedds' / 'include'}")
    cg.add_build_flag(f"-I{third_party / 'microcdr' / 'include'}")
    cg.add(var.set_agent_address(config[CONF_AGENT_ADDRESS]))
    cg.add(var.set_agent_port(config[CONF_AGENT_PORT]))
    cg.add(var.set_domain_id(config[CONF_DOMAIN_ID]))
    cg.add(var.set_client_name(config[CONF_CLIENT_NAME]))
    cg.add(var.set_max_packet_length(config[CONF_MAX_PACKET_LENGTH]))
    cg.add(var.set_process_interval(config[CONF_PROCESS_INTERVAL]))
    cg.add(var.set_keepalive_timeout(config[CONF_KEEPALIVE_TIMEOUT]))
    cg.add(var.set_max_topics(config[CONF_MAX_TOPICS]))
    cg.add(var.set_max_datawriters(config[CONF_MAX_DATAWRITERS]))
    cg.add(var.set_max_datareaders(config[CONF_MAX_DATAREADERS]))
    transport = config[CONF_TRANSPORT]
    if transport[CONF_TRANSPORT_TYPE] == "udp":
        cg.add(var.set_transport_udp())
    else:
        parent = await cg.get_variable(transport[CONF_UART_ID])
        cg.add(var.set_transport_serial(parent))
