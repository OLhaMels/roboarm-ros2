# Raspberry Pi 5 two-axis ROS 2 arm

ROS 2 **Jazzy on Ubuntu 24.04 arm64**, with a vertical A4988 motor and an A4988 motor rotating a fixed-length horizontal arm. The third motor is unused. The gripper action is simulated. The existing `web_test_motor.py` is preserved as a reference; do not run it alongside this system.

Implemented: ROS packages, configurable robot model, ros2_control GPIO plugin, MoveIt IK plugin and planning configuration, task manager, camera and marker perception nodes, and Web UI. **Default mode is mock hardware. This workspace has passed offline tests, but the ROS build, complete runtime, and physical GPIO timing have not been verified here.** Run the included Jazzy build and smoke test before commissioning.

## Documentation

Start with the [setup and operation guide](docs/setup-and-operation.md). It takes you from installing ROS on the Pi to building, launching, using the webpage, configuring motors, stopping, and troubleshooting.

| Document | What it explains |
|---|---|
| [Setup and operation](docs/setup-and-operation.md) | Installation, first simulated move, physical launch, camera setup, daily operation and troubleshooting |
| [Configuration reference](docs/configuration.md) | Every launch option and hardware setting, units, GPIO requirements and calibration calculations |
| [Architecture and communication](docs/architecture.md) | How the project works, each node, the original motor script, and all diagram topics/services/actions |

All shell commands in these guides run in **Bash on the Ubuntu Raspberry Pi**, unless explicitly labelled as commands for your computer. They are not Windows PowerShell commands.

## Model and scope

- `z_joint`: metres of vertical carriage travel.
- `rotation_joint`: radians of arm rotation around Z. The webpage motion control displays degrees.
- `gripper_joint`: simulated finger displacement in metres.
- No homing switches or encoders are assumed. Physical joint states estimate emitted pulses; they do not measure the arm.
- Travel limits, initial reference positions, geometry, calibration, and GPIO assignments are editable on the webpage. Saved settings apply **after restarting bringup**, consistently to URDF, hardware, task validation, and MoveIt.
- No physical pick/place is offered without a gripper. The GPIO arm supports two-axis motion; `/robot_arm/pick_place` rejects goals in GPIO mode. The gripper action remains available as a simulation.

With arm radius `R`, base height `H`, carriage position `z`, and rotation `theta`:

```text
x = R * cos(theta)
y = R * sin(theta)
tool_height = H + z
tool_yaw = theta
```

The reachable workspace is a bounded cylindrical surface. Two motors cannot independently choose X, Y, and Z, or an arbitrary tool orientation. The IK plugin rejects poses outside this geometry instead of silently projecting them onto it. Supplied geometry and travel limits are examples, not measurements from the photograph.

See [the diagram analysis and exact ROS interfaces](docs/architecture.md).

## Install and build on the Pi

Install Ubuntu 24.04 **64-bit** and ROS 2 Jazzy using the [official ROS installation instructions](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html). In this repository directory:

```bash
source /opt/ros/jazzy/setup.bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep \
  python3-pytest libgpiod-dev gpiod
# Run sudo rosdep init once, only if rosdep has not already been initialized.
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy \
  --skip-keys libgpiod-dev
colcon build --symlink-install --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

`libgpiod-dev` is installed explicitly and skipped in rosdep to avoid distribution-specific key availability. This driver uses the **libgpiod 1.x C API shipped with Ubuntu 24.04**, not the incompatible 2.x API. Use sequential builds to reduce peak memory on the Pi.

## Start the simulated system

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch roboarm_bringup bringup.launch.py
```

Open **http://127.0.0.1:5000** on the Pi. From another computer, use an SSH tunnel:

```bash
ssh -L 5000:127.0.0.1:5000 USER@PI_ADDRESS
```

Then open http://127.0.0.1:5000 locally. The UI is unauthenticated and defaults to loopback. `web_host:=0.0.0.0` exposes it on the network and should only be used on a controlled network.

The default launch creates a user-editable configuration at `~/.config/roboarm/hardware.yaml`, copied from the example on first use. A custom file can be selected with `hardware_config:=/absolute/path/hardware.yaml`. The UI shows the saved file location and currently active limits.

The page provides absolute Z/angle moves through MoveIt, cancellation, simulated pick/place, an optional simulated vision target, configuration editing, controller listing, and controller activation/deactivation. Long-running motion returns immediately to the browser and reports action feedback/results. Pick/place and manual moves share one task-manager motion lock.

The mock perception path labels its images as simulated and emits a demonstration target. It does not run object recognition. `demo_perception:=false` enables the configured camera backend and QR label detection instead.

## Configure actual motors

1. Start in mock mode. On the webpage, set the measured arm radius, base height, travel limits, motor calibration, speed limits, GPIO chip, and GPIO assignments. Set initial positions to coordinates you can physically establish. Save and stop bringup.
2. Verify the Pi's RP1 GPIO device with `gpiodetect` and `gpioinfo`. Set its device path; `/dev/gpiochip4` is only an example. BCM offsets belong to that chip, not physical header pin numbers. Give the ROS process read/write access using the OS GPIO group/device policy; do not run the complete application as root.
3. For **each driver**, assign STEP, DIR, MS1, MS2, MS3, and active-low `/ENABLE`. The Z driver's STEP/DIR/MS assignments initially match the original script; its enable line and all rotation-driver lines remain unassigned. Optional limit-switch pins stay `-1` for the current hardware.
4. Full-step mode is fixed: MS1/MS2/MS3 are driven LOW. Connect A4988 logic ground to Pi ground, use a suitable separate motor supply, correctly handle RESET/SLEEP per your carrier, and set the driver's current limit for the motor. The enable input should have an external pull-up so the driver defaults to disabled when the Pi has not claimed GPIO.
5. Confirm motor direction with a small unloaded move during commissioning and adjust `direction_inverted`. In ROS, positive rotation is counterclockwise viewed from +Z; the actual DIR level depends on wiring. Positive Z must raise the arm.
6. Set `commissioned: true` only after entering actual calibration and assignments. Place both axes at their configured `initial_position`. Without switches/encoders, this is a **manual reference**, not automatic homing.
7. Start physical mode with explicit reference confirmation:

```bash
ros2 launch roboarm_bringup bringup.launch.py \
  hardware_mode:=gpio demo_perception:=false reference_confirmed:=true
```

Reference confirmation defaults to false **on every launch**, regardless of the saved YAML. After a restart, power loss, deactivation, stall or suspected missed step, re-establish the reference. The webpage cannot discover the physical position. To change the initial coordinates, save them in the webpage, stop the system, establish those coordinates manually, then relaunch with reference confirmation.

Calibration:

```text
Z steps_per_unit        = full_steps_per_motor_rev * gear_ratio / screw_lead_metres
Rotation steps_per_unit = full_steps_per_motor_rev * gear_ratio / (2*pi)
```

For a belt-driven Z axis, replace screw lead with linear travel per output revolution. `gear_ratio` is motor revolutions per output revolution. Do not use the example scale without measuring the mechanism. Software travel limits are only as accurate as this calibration and the initial reference.

The GPIO worker issues at most 500 steps/s per joint (default 400), with direction setup, pulse-width delays, a conservative ramp, software bounds and a controller-write watchdog. `read()` reports pulse estimates; `write()` updates targets without generating a blocking 600-step burst. Linux scheduling can stretch pulse intervals, so this is a low-speed starting point, not a hard-real-time pulse generator. Use a dedicated MCU for higher speeds or tightly synchronized motion; no MCU protocol or firmware is invented here.

Stopping the GPIO system or a fault disables the drivers. A vertical axis that can fall when torque is removed needs mechanical support/braking. The webpage Cancel button and software limits do not replace a physical stop or prevent movement after lost steps. No switch is required by this implementation; optional NC-to-ground switches, if later fitted, require external 3.3 V pull-ups and latch a fault when opened. There is no automatic homing routine.

## Camera and vision

`camera_node` publishes `/camera/image_raw` and `/camera/camera_info` from one of several backends: `libcamera` (CSI IMX219 through GStreamer `libcamerasrc`, the Yocto image's camera path), `picamera2`, `v4l2` (USB via OpenCV) or `images` (photo files, for testing without a camera).

`vision_node` reads QR labels with zbar (the scanner from upd-rasp-stream-tcp). Every new read is published on `/vision/qr`; part links are followed to their iFurn page and appended to `parts.csv` (also published on `/vision/part`). A code whose payload matches `target_pattern` and whose printed size is `qr_size` is located with `solvePnP` and transformed into `base_link` using TF at the image timestamp, then published on `/vision/target_pose`. Publish a measured transform from `base_link` to `camera_optical_frame`; no unmeasured camera transform is supplied. Zero focal lengths deliberately prevent target publication; calibration must match the capture resolution.

`video_stream:=true` adds `stream_node`, which serves the annotated `/vision/debug_image` as low-latency H.264 MPEG-TS on TCP port 8554 (`ffplay -fflags nobuffer -flags low_delay -framedrop tcp://PI_ADDRESS:8554`). It needs `sudo apt install python3-av`.

Details, testing without hardware, and the Yocto recipe: [docs/perception-qr.md](docs/perception-qr.md).

## Verify ROS communication

With bringup running:

```bash
ros2 topic list -t
ros2 action list -t
ros2 service list -t
ros2 topic echo /joint_states --once
ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers '{}'
ros2 action send_goal /robot_arm/move_joints roboarm_interfaces/action/MoveJoints \
  '{z: 0.06, rotation: 0.2}' --feedback
```

Joint commands in metres/radians enter MoveIt via the task manager. The example moves both axes; inspect your active limits first.

Run package/offline tests:

```bash
python3 -m pytest src/roboarm_nodes/test tests/test_description.py -q
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

Stop any existing bringup, then run the full **mock-only** smoke test in a separate domain:

```bash
ROS_DOMAIN_ID=83 python3 tests/ros_smoke.py
```

It launches its own mock system and checks diagram topics/services, active controllers, reachable/unreachable IK, invalid targets, two-axis motion, simulated pick/place, cancellation, and controller switching. Logs go to `ros_smoke.log`. The GitHub Actions workflow builds and runs these checks on Jazzy; it has been added but has not been run from this workspace.

## Remaining physical integration

The user still needs to provide actual GPIO assignments, geometry, transmission ratios, motor current settings, reference procedure, and measured limits. Physical timing/load tests and a Raspberry Pi build are required. Collision geometry is illustrative; add measured geometry and obstacles before relying on collision planning. No carried-object collision model is attached during the simulated pick/place sequence. A later physical gripper requires replacing `ExampleGripper` with its real hardware interface and updating the task-manager physical pick/place gate.
