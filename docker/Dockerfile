# Perception layer of the roboarm (camera_node, vision_node, stream_node) for
# a Raspberry Pi 5 with a CSI camera, on ROS 2 Jazzy.
#
# Ubuntu's own libcamera has no Pi 5 (PiSP) pipeline, so Raspberry Pi's fork
# of libcamera and libpisp are built here, with the GStreamer libcamerasrc
# element that camera_node's `libcamera` backend uses. Build from the
# repository root:
#
#   docker compose -f docker/docker-compose.yml build
FROM ros:jazzy-ros-base

ARG LIBPISP_VERSION=v1.7.0
ARG LIBCAMERA_VERSION=v0.7.2+rpt20261009

ENV DEBIAN_FRONTEND=noninteractive PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1

RUN apt-get update && apt-get install -y --no-install-recommends \
        git meson ninja-build pkg-config cmake g++ \
        python3-yaml python3-ply python3-jinja2 \
        libyaml-dev libgnutls28-dev libdw-dev libudev-dev libboost-dev nlohmann-json3-dev \
        libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
        gstreamer1.0-tools gstreamer1.0-plugins-base gir1.2-gstreamer-1.0 python3-gi \
        python3-numpy python3-pil python3-opencv python3-av libzbar0t64 fonts-dejavu-core \
        ros-jazzy-tf2-ros-py ros-jazzy-tf2-geometry-msgs ros-jazzy-rmw-cyclonedds-cpp \
        python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

RUN git clone --depth 1 --branch ${LIBPISP_VERSION} https://github.com/raspberrypi/libpisp.git /tmp/libpisp \
    && meson setup /tmp/libpisp/build /tmp/libpisp --prefix=/usr/local --libdir=lib --buildtype=release \
        -Dlogging=disabled \
    && ninja -C /tmp/libpisp/build install \
    && git clone --depth 1 --branch ${LIBCAMERA_VERSION} https://github.com/raspberrypi/libcamera.git /tmp/libcamera \
    && meson setup /tmp/libcamera/build /tmp/libcamera --prefix=/usr/local --libdir=lib --buildtype=release \
        -Dpipelines=rpi/pisp -Dipas=rpi/pisp -Dgstreamer=enabled -Dv4l2=false -Dpycamera=disabled \
        -Dcam=disabled -Dqcam=disabled -Dlc-compliance=disabled -Ddocumentation=disabled -Dtest=false \
    && ninja -C /tmp/libcamera/build install \
    && ldconfig && rm -rf /tmp/libpisp /tmp/libcamera

ENV GST_PLUGIN_PATH=/usr/local/lib/gstreamer-1.0 \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

WORKDIR /ws
COPY src/roboarm_nodes src/roboarm_nodes
COPY src/roboarm_bringup src/roboarm_bringup
RUN . /opt/ros/jazzy/setup.sh \
    && colcon build --packages-select roboarm_nodes roboarm_bringup \
    && rm -rf build log

COPY --chmod=755 docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "roboarm_bringup", "perception.launch.py", \
     "perception_config:=/config/perception.yaml", "video_stream:=true"]
