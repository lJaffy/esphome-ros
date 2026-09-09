# Prebuilt agent (Ubuntu 24.04 / ROS Jazzy). Building MicroXRCEAgent from
# source is pinned to a Fast-DDS branch (2.12.x) that no longer resolves
# upstream, so the superbuild cannot complete — use this image instead.
FROM microros/micro-ros-agent:jazzy

ENV ROS_DOMAIN_ID=0

# Only python3-pip is missing from the base image (rclpy + std_srvs
# ship with it — verified via `dpkg -s`). No ROS packages are installed
# here: reinstalling them on top of the prebuilt image caused FastDDS
# version skew inside the jazzy stack (newer libstd_srvs against older
# libfastcdr), breaking `ros2 service call` with undefined-symbol errors
# while the rclpy server itself still listed fine.
RUN apt-get update && apt-get install -y --no-install-recommends python3-pip \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/capture
COPY requirements.txt .
RUN pip3 install --no-cache-dir --break-system-packages -r requirements.txt
COPY capture.py dds_types.py trigger_server.py entrypoint.sh ./
RUN chmod +x /opt/capture/entrypoint.sh

# UDP port the S3 dials (match xrce_dds: agent_port).
EXPOSE 8888/udp

# Run directory for `record` (frame-*.jpg + index.jsonl). Mount it:
#   docker run --rm --net=host -v "$PWD/runs:/out" s3-dds-capture
VOLUME ["/out"]
ENV OUT_DIR=/out
ENV AGENT_PORT=8888

# The agent executable lives in ROS libexec space (not on PATH), so expose
# it for `capture.py agent` and for direct use. LD_LIBRARY_PATH covers
# running it without sourcing the ROS workspace first.
RUN AGENT_EXE=$(find /uros_ws/install -path "*lib/micro_ros_agent/micro_ros_agent" -type f | head -n 1) && \
    test -n "$AGENT_EXE" && \
    ln -s "$AGENT_EXE" /usr/local/bin/micro-ros-agent
# Whole workspace on the loader path (agent + micro_ros_msgs + ROS libs
# resolve without sourcing a shell first).
RUN for d in /uros_ws/install/*/lib /opt/ros/jazzy/lib; do echo "$d"; done \
      > /etc/ld.so.conf.d/micro-ros.conf && ldconfig
# FastDDS skew guard: libstd_srvs' fastrtps typesupport must actually load
# against the installed libfastcdr. A version skew inside the jazzy stack
# (newer std_srvs against older fastcdr, or vice versa) surfaces only at
# runtime as `undefined symbol: _ZN8eprosima7fastcdr3Cdr9serializeERKh` —
# while `service list` still works. Creating a Trigger client dlopens that
# typesupport, so this fails the build loudly instead of the board timing
# out with no reply at HIL time. `set +u`: the ROS setup scripts reference
# possibly-unset vars (AMENT_TRACE_SETUP_FILES).
# NOTE: bare --help exits 1 by design (it demands a transport), so assert
# on output: proves the loader resolves all agent libs, which is what we need.
RUN micro-ros-agent --help 2>&1 | grep -q "udp4"
RUN set +u; . /opt/ros/jazzy/setup.sh; set -u; python3 -c \
    "import rclpy; rclpy.init(); \
from rclpy.node import Node; from std_srvs.srv import Trigger; \
n = Node('build_smoke'); c = n.create_client(Trigger, '/build_smoke'); \
n.destroy_node(); rclpy.shutdown(); print('Trigger typesupport loads OK')"

# Replaces the base image's entrypoint (which execs the agent): bare
# `docker run` starts the agent AND records to $OUT_DIR. Pass an explicit
# command to run just one tool, e.g. `docker run ... s3-dds-capture
# python3 /opt/capture/capture.py discover`.
ENTRYPOINT ["/opt/capture/entrypoint.sh"]
CMD []
