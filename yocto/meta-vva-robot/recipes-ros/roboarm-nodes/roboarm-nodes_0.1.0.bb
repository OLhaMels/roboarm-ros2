# roboarm_nodes for the vva-robot image: camera_node, vision_node (QR) and
# the optional stream_node. Drop this directory into meta-vva-robot and add
# "roboarm-nodes" to the image (IMAGE_INSTALL:append = " roboarm-nodes").
#
# Only the perception nodes' runtime dependencies are listed. task_manager,
# web_ui also need roboarm-interfaces, moveit-msgs, control-msgs,
# controller-manager-msgs and python3-flask; add them when the arm side moves
# onto the image too.

SUMMARY = "Roboarm camera, QR vision and video stream ROS 2 nodes"
# Not published under an open-source license.
LICENSE = "CLOSED"

ROS_CN = "roboarm_nodes"
ROS_BPN = "roboarm_nodes"

# Point this at the repository once it is pushed, e.g.
#   SRC_URI = "git://github.com/<you>/roboarm-ros2.git;protocol=https;branch=main;subdir=git"
#   SRCREV = "<commit>"
#   S = "${WORKDIR}/git/src/roboarm_nodes"
# Until then the package is copied in as local files:
SRC_URI = "file://roboarm_nodes \
           file://vva-vision.service \
           file://vva-stream.service \
"
S = "${WORKDIR}/roboarm_nodes"

ROS_BUILD_DEPENDS = ""
ROS_BUILDTOOL_DEPENDS = ""
ROS_EXPORT_DEPENDS = ""
ROS_BUILDTOOL_EXPORT_DEPENDS = ""

ROS_EXEC_DEPENDS = " \
    rclpy \
    sensor-msgs \
    geometry-msgs \
    std-msgs \
    tf2-ros-py \
    tf2-geometry-msgs \
    python3-numpy \
    python3-pillow \
    zbar \
    python3-opencv \
    python3-pygobject \
    gstreamer1.0 \
    gstreamer1.0-plugins-base-videoconvert \
    gstreamer1.0-plugins-base-app \
    libcamera-gst \
"
# stream_node only. PyAV is not in meta-python for Scarthgap; without it the
# other nodes still run and stream_node exits with an ImportError.
RRECOMMENDS:${PN} += "python3-av"

DEPENDS = "${ROS_BUILD_DEPENDS} ${ROS_BUILDTOOL_DEPENDS}"
DEPENDS += "${ROS_EXPORT_DEPENDS} ${ROS_BUILDTOOL_EXPORT_DEPENDS}"
RDEPENDS:${PN} += "${ROS_EXEC_DEPENDS}"

ROS_BUILD_TYPE = "ament_python"

inherit ros_distro_${ROS_DISTRO}
inherit ros_${ROS_BUILD_TYPE}
inherit systemd

SYSTEMD_SERVICE:${PN} = "vva-vision.service vva-stream.service"
# Like vva-camera.service and vva-motor.service: installed, enabled by hand
# after the camera has been checked on the bench.
SYSTEMD_AUTO_ENABLE = "disable"

do_install:append() {
    install -d ${D}${systemd_system_unitdir}
    install -m 0644 ${WORKDIR}/vva-vision.service ${WORKDIR}/vva-stream.service ${D}${systemd_system_unitdir}/
}

FILES:${PN} += "${systemd_system_unitdir}"
