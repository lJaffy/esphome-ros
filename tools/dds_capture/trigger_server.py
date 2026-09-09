#!/usr/bin/env python3
"""Dev-exercise Trigger service server for the Phase-3 service spike.

Serves fixed /toggle_led (std_srvs/Trigger) so the board's service
client gets a reply instead of a timeout. Needs a ROS 2 install
(rclpy + std_srvs) — the dds_capture image provides both.

rclpy is imported lazily inside main() so this module stays
importable (and unit-testable) on machines without ROS 2.

Run on the same DDS domain/net as the agent:
    python3 trigger_server.py
    # or: docker compose up trigger

Verify: ros2 service list | grep toggle_led
"""

SERVICE_NAME = "/toggle_led"


def build_reply_text(call_count):
    """Pure helper (stdlib-only, unit-tested): reply message for call N."""
    return "toggle_led ok #%d" % call_count


def main():
    import rclpy
    from rclpy.node import Node
    from std_srvs.srv import Trigger

    rclpy.init()
    node = Node("toggle_led_server")
    state = {"calls": 0}

    def on_trigger(request, response):
        del request  # Trigger has an empty request.
        state["calls"] += 1
        response.success = True
        response.message = build_reply_text(state["calls"])
        node.get_logger().info(
            "toggle_led call #%d -> success=true" % state["calls"])
        return response

    node.create_service(Trigger, SERVICE_NAME, on_trigger)
    node.get_logger().info("serving %s (std_srvs/Trigger)" % SERVICE_NAME)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
