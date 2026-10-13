# Setup and operation guide

[Documentation home](../README.md) · [Settings reference](configuration.md) · [Architecture and ROS interfaces](architecture.md)

This guide explains how to install, build and use the project on a Raspberry Pi 5 running **Ubuntu 24.04 arm64 and ROS 2 Jazzy**. The Pi drives two A4988 motors: one moves the arm vertically, and one rotates the horizontal arm around the vertical column. The third motor is unused and the gripper is simulated.

The project currently has offline validation, not a verified complete ROS build or physical commissioning. Installation and runtime commands below are the intended workflow; run the included checks and inspect failures before connecting movement to a loaded arm.

## 1. Understand what will run

The webpage sends an absolute Z position and rotation angle to `task_manager_node`. The task manager validates the request and asks MoveIt to plan a motion. MoveIt sends a timed trajectory to `arm_controller`. The controller periodically provides joint positions to the hardware plugin, which either updates mock state or produces STEP/DIR pulses for the A4988 drivers.

State travels back through `joint_state_broadcaster` to `/joint_states`. MoveIt uses those states for planning, `robot_state_publisher` calculates the link transforms, and the webpage displays the current positions. On real hardware, these positions are estimates from emitted pulses, not encoder measurements.

The camera and vision nodes form a separate path: camera images → marker detection → target pose → task manager. Receiving a vision target **does not automatically start motion**. It is used only when an explicit simulated pick/place goal requests it.

| Mode | Arm motors | Gripper | Camera / vision | Intended use |
|---|---|---|---|---|
| Default `mock` | No GPIO access; state follows commands | Simulated | Synthetic images and target | Learn the UI and check communication without motors |
| `mock` with real perception | No GPIO access | Simulated | Camera (libcamera, picamera2 or USB), QR detection | Test camera calibration and target coordinates |
| `gpio` | Physical Z and rotation motors | Still simulated | Real perception mode required by launch | Move the two physical axes |

Physical pick/place is rejected because there is no real gripper. The mock system does not include gravity, contact forces or grasp physics. There is no RViz window launched by default.

The full component and endpoint tables are in [Architecture and communication](architecture.md).

## 2. Prepare Ubuntu and ROS 2

Use Ubuntu **24.04 64-bit**, not Raspberry Pi OS for these commands. On the Pi, open a terminal and check:

```bash
cat /etc/os-release
uname -m
```

The OS should identify as Ubuntu 24.04 and the architecture as `aarch64`. Internet access is needed for dependencies. A desktop is optional; the UI works from another computer through SSH.

Install ROS 2 Jazzy by following the **locale and repository configuration** steps in the [official Jazzy Ubuntu installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html). The [official source version](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Installation/Ubuntu-Install-Debs.rst) is available if the documentation website is inaccessible. Set up the ROS repository before running the next commands; otherwise apt cannot find `ros-jazzy-*` packages.

For this project's browser UI, ROS Base is sufficient; project-specific dependencies are installed in the next section:

```bash
sudo apt update
sudo apt install ros-jazzy-ros-base build-essential \
  python3-colcon-common-extensions python3-rosdep python3-pytest \
  libgpiod-dev gpiod
source /opt/ros/jazzy/setup.bash
printenv ROS_DISTRO
ros2 --help
```

`ROS_DISTRO` should print `jazzy`. Use the system `python3` with ROS's apt packages, rather than an unrelated Conda or virtual environment. The driver targets Ubuntu 24.04's libgpiod **1.x API**; replacing it with libgpiod 2.x requires code changes.

## 3. Copy the repository to the Pi

The following examples assume the project is at `~/roboarm-ros2` on the Pi. Copy the repository directory there using Git, an SFTP client, or a removable drive. Include hidden files if you want the CI configuration as well. No repository URL is assumed by this guide.

Copy source files, not Windows build products. The required layout is:

```text
roboarm-ros2/
  src/
    roboarm_interfaces/   custom task actions
    roboarm_nodes/        task manager, camera, vision, webpage
    roboarm_hardware/     A4988 ros2_control plugin
    roboarm_kinematics/   two-axis MoveIt inverse kinematics
    roboarm_bringup/      launch, robot model and configuration
  tests/
  docs/
  README.md
  web_test_motor.py       original reference script; not used by ROS
```

There is no need to copy `.validation_deps`, `__pycache__`, `build`, `install`, or `log`. Do not run the original Flask motor script while ROS owns the GPIO.

## 4. Install project dependencies and build

Run on the Pi:

```bash
cd ~/roboarm-ros2
source /opt/ros/jazzy/setup.bash
```

If this machine has never used rosdep, initialize it once:

```bash
sudo rosdep init
```

If rosdep says its default sources file already exists, it is already initialized. Continue with the update as your normal user:

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy \
  --skip-keys libgpiod-dev
colcon build --symlink-install --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
ros2 pkg prefix roboarm_bringup
```

`libgpiod-dev` is skipped by rosdep because it was installed explicitly. The package lookup should return a directory inside this workspace's `install` directory. Do not proceed after a failed build. `build/`, `install/` and `log/` are generated by colcon; `src/` is the source code.

If compilation is killed because of memory pressure, repeat the build with `CMAKE_BUILD_PARALLEL_LEVEL=1` prefixed to the `colcon build` command. Sequential package builds alone do not limit compiler parallelism within each package.

Every new terminal used for this project needs both environments:

```bash
cd ~/roboarm-ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## 5. Launch without moving motors

Start the default mock system:

```bash
ros2 launch roboarm_bringup bringup.launch.py
```

Leave this terminal running. Launch starts the robot-state publisher and controller manager, then activates the joint-state broadcaster, arm controller and simulated gripper controller. The application nodes start only after the controller spawners succeed. A spawner failure stops launch; errors appear in this terminal.

On the first launch, the example hardware settings are copied to `~/.config/roboarm/hardware.yaml`. Later launches reuse that file; changes to the repository's example YAML do not overwrite it.

Open [http://127.0.0.1:5000](http://127.0.0.1:5000) in a browser on the Pi. To use a browser on your computer, run this in a terminal **on your computer**, replacing the username and address:

```bash
ssh -L 5000:127.0.0.1:5000 YOUR_PI_USER@YOUR_PI_ADDRESS
```

Keep the SSH session open and visit the same localhost URL on your computer. The connection forwards your local port 5000 to the Pi's port 5000. If that local port is already used, use `-L 5001:127.0.0.1:5000` and open localhost port 5001 instead.

Wait for the page to show arm mode `mock` and fresh status. With the supplied defaults, positions start near Z `0.05` m and rotation `0` rad. The Controllers panel should show these as `active`:

- `joint_state_broadcaster`
- `arm_controller`
- `gripper_controller`

## 6. Try the webpage

### Absolute motion

In mock mode with unchanged example settings:

1. Enter Z `0.06` metres and angle `10` degrees in **Motion**.
2. Click **Move to position**.
3. Watch the task state and job result. A successful job ends with `success: true`.
4. To interrupt an active goal, click **Cancel motion** and wait for its terminal result.

The inputs are **absolute positions**. Entering Z `0.06` means move to the 60 mm coordinate, not move upwards by 60 mm. The motion form uses degrees, but ROS messages and the configuration's rotational limits use radians. Displayed joint-state rotation is also in radians.

The arm's X/Y coordinates follow its fixed radius. Changing the angle moves the tool along a circle; there is no independent X/Y translation motor.

### Simulated pick/place

Leave **Use vision pick target** unchecked for the first test. With the example configuration, the form's default pick/place positions and clearance fit the limits. Click **Run simulation** and watch the stages: open, approach, lower, close, lift, transfer, lower, release and retreat.

Later, enable the vision checkbox to use the current synthetic target as the pick location. The place location still comes from the form. If you change limits or geometry, check that the demonstration target and both clearance poses are still reachable. A fresh image/target is required.

### Save settings

Edit the **Configuration** section only while idle. **Save for next startup** writes the settings to disk. It does not change the running robot model or active limits. **Reload saved settings** reloads the form from disk; it does not apply settings to ROS.

To apply a saved change:

1. Finish or cancel the active motion.
2. Stop launch with `Ctrl+C` in its terminal.
3. Launch again using the same configuration file.
4. Check the active limits displayed on the page.

## 7. Configure the physical motors

Use the [configuration reference](configuration.md) while filling in the page. Start in mock mode so incomplete wiring settings cannot energize the motors.

Measure the arm radius, the base-to-tool height at Z zero, the usable travel, and the transmission ratios. Establish a repeatable coordinate reference for both joints. Set conservative travel bounds that lie inside mechanical clearance. Do not infer those values from the photograph or example YAML.

For each A4988, this implementation requires six output lines: DIR, STEP, MS1, MS2, MS3 and active-low ENABLE. The Z STEP/DIR/MS values match the original script; the Z ENABLE and all rotation-driver outputs still need assignment. No homing or travel switches are required; leave the optional switch fields at `-1`.

Use these read-only commands on the Pi to inspect GPIO:

```bash
gpiodetect
gpioinfo
ls -l /dev/gpiochip*
id
```

Identify the RP1 chip that exposes the header GPIO and enter its device path. Do not assume that `/dev/gpiochip4` is correct for every kernel. The pin fields use BCM GPIO offsets on that chip, not physical header numbers. Grant your normal user access through the OS's GPIO device/group policy. Group membership changes may require logging out and back in. Do not work around permissions by launching the whole stack as root or making GPIO devices world-writable.

MS1–MS3 are driven LOW for full stepping. Configure the carrier's RESET/SLEEP, logic supply, current limit and motor supply according to the carrier and motor specifications. Pi GPIO is 3.3 V logic; never connect motor supply voltage to GPIO. Provide a shared logic ground and an external enable pull-up so the driver remains disabled before the software claims the pins. Keep the vertical carriage mechanically supported if losing holding torque would let it fall.

Before a first powered direction test, enter the measured settings, assign all required lines, set low speeds and a small travel interval, then mark `commissioned` true. This flag is your acknowledgement of the configuration; it is not a test performed by the software. Save and stop the mock system.

### Manual position reference

There is no automatic homing and no way for software to discover where the motors are after power-up. With the mechanism supported and drivers appropriately disabled, establish both configured `initial_position` coordinates manually. Never force an energized mechanism to establish its reference.

For example, an initial Z value of `0.05` means the carriage must physically be at the chosen 50 mm coordinate when launch begins. It does not instruct the robot to search for or move to that coordinate. Repeat the reference after loss of position, power interruption or restart.

### Launch physical motion

After establishing both reference positions:

```bash
cd ~/roboarm-ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch roboarm_bringup bringup.launch.py \
  hardware_mode:=gpio \
  demo_perception:=false \
  reference_confirmed:=true
```

GPIO startup rejects missing required pins, duplicate pin assignments, invalid calibration, an unset `commissioned` flag or missing reference confirmation. `reference_confirmed` must be supplied each startup; saving a true value in YAML cannot bypass this requirement.

The real-perception setting makes the camera node attempt to open a USB camera. If none is present, the camera node can exit with an error while the independently launched arm stack remains available; the current launch has no camera-disable argument. Camera availability is not required for direct two-axis motion.

Confirm the webpage says `gpio`. Make a small movement of one axis at a time, holding the other input at its current coordinate. Positive Z must raise the arm; positive rotation follows the ROS +Z rotation convention. If the direction is wrong, stop, adjust `direction_inverted`, save, re-establish the reference and restart. Do not continue a larger move to diagnose direction.

The gripper remains simulated in this mode, and physical pick/place requests are rejected. **Cancel motion** cancels the current webpage goal; it is not an electrical emergency stop and does not cancel unrelated goals submitted by other ROS clients.

## 8. Add a real camera when needed

First verify the camera with the arm still in mock mode. Copy the perception settings to a writable location:

```bash
mkdir -p ~/.config/roboarm
cp src/roboarm_bringup/config/perception.yaml ~/.config/roboarm/perception.yaml
```

Edit that file with the camera `backend` (`libcamera` for the CSI IMX219, `v4l2` plus `device` for USB), actual image size, camera intrinsics/distortion and the printed QR size `qr_size`. The example focal lengths are zero, which intentionally prevents metric target publication.

Provide a **measured** TF transform between `base_link` and `camera_optical_frame`; no camera mounting transform is assumed. Optical coordinates use X right, Y down and Z forward. Calibration must match the camera position and image resolution. Use your calibration tool to publish the transform, or provide its measured values to a static transform publisher.

Launch a mock arm with real perception:

```bash
ros2 launch roboarm_bringup bringup.launch.py \
  hardware_mode:=mock demo_perception:=false \
  perception_config:=$HOME/.config/roboarm/perception.yaml
```

The detector reads QR labels with zbar. Only payloads matching `target_pattern` become targets; it publishes the code centre after calibration and TF conversion. Every new read also appears on `/vision/qr`, and part links are recorded to `parts.csv` (see [perception-qr.md](perception-qr.md)). The optional `marker_to_tool_z` parameter adds a base-frame vertical offset. Object orientation is not reproduced; tool yaw follows the arm's radial direction.

Check output from a second sourced terminal:

```bash
ros2 topic hz /camera/image_raw
ros2 topic echo /camera/camera_info --once
ros2 topic echo /vision/target_pose --once
```

These commands may keep waiting if no data is published; stop them with `Ctrl+C`. An off-circle vision target can be detected correctly but still be unreachable by this two-axis arm.

## 9. Verify the system and stop it

Inspect the graph from a second terminal after sourcing both environments:

```bash
ros2 node list
ros2 topic list -t
ros2 action list -t
ros2 service list -t
ros2 topic echo /joint_states --once
ros2 service call /controller_manager/list_controllers \
  controller_manager_msgs/srv/ListControllers '{}'
```

Use the [interface map](architecture.md#exact-communication-map) to compare the results with the original diagram. Built-in controller nodes and plugin internals need not match Python filenames one-to-one.

Run the offline and package tests after building:

```bash
python3 -m pytest src/roboarm_nodes/test tests/test_description.py -q
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

Stop other bringup instances before running the integration smoke test; a different ROS domain does not isolate the Web UI's TCP port:

```bash
ROS_DOMAIN_ID=83 python3 tests/ros_smoke.py
```

The smoke test starts its own **mock-only** launch and exercises communication, IK, movement, cancellation, simulated pick/place and controller switching. It writes `ros_smoke.log`. The test's domain ID isolates its ROS graph from clients using another domain.

For normal shutdown, finish or cancel motion, verify the mechanism is supported, and press `Ctrl+C` in the launch terminal. Physical hardware shutdown disables the drivers and removes holding torque. Controller **Deactivate** in the webpage is a controller-management operation, not full hardware shutdown or an emergency stop. A latched GPIO fault requires stopping the stack, addressing the cause and re-referencing before a new launch.

After source-code changes, stop the stack, rebuild with colcon, source `install/setup.bash` again and relaunch. C++ plugins must be rebuilt. Runtime settings saved from the webpage only need a restart.

## 10. Troubleshooting

| Symptom | Check / next step |
|---|---|
| `ros2: command not found` | Source `/opt/ros/jazzy/setup.bash`; confirm Jazzy is installed. |
| `Package 'roboarm_bringup' not found` | Check that colcon finished successfully and source this workspace's `install/setup.bash`. |
| Apt cannot find ROS packages | Complete the official ROS repository setup before installing Jazzy. |
| Build killed or compiler terminated | Reduce compiler parallelism with `CMAKE_BUILD_PARALLEL_LEVEL=1` and inspect memory/storage availability. |
| GPIO API compile errors | Check libgpiod version; the current driver uses 1.x functions. |
| Spawner times out; no webpage | Read the first controller-manager error. Check robot description, plugin loading and configuration. Application startup waits for controllers. |
| Webpage does not open | Keep launch running; check its web-server log. For remote access, verify the SSH tunnel and local port. |
| `Address already in use` | Stop the previous process using port 5000; do not run the old `web_test_motor.py` at the same time. |
| Status stale or task rejected | Confirm task manager and joint-state broadcaster are running and the requested target is inside active limits. |
| Settings saved but behaviour unchanged | Restart bringup with the same hardware file; saved settings do not update the active model. |
| Configured velocity rejected | Check the step-rate/scale capacity formula in the settings reference. |
| Cannot open/claim GPIO | Verify chip path, user access, unique assignments, and that another program does not own the lines. |
| Missing pin or commissioning error | Assign both motors' required output lines; mark commissioning after configuration. Optional switches may remain `-1`. |
| Missing reference confirmation | Manually establish the configured positions and supply `reference_confirmed:=true` on this launch. |
| Motion reports success but physical displacement is wrong | Stop and check calibration, gearing, direction, current limit and lost steps. Joint states are not encoder feedback. |
| Task fault says stop or acceptance is uncertain | Do not resubmit blindly. Verify motion has stopped, address the cause, restart and re-reference hardware. |
| Gripper does not physically move | Expected: the gripper is simulated and no third motor is driven. |
| Pick/place rejected in GPIO mode | Expected until a real gripper is integrated. Use direct two-axis motion. |
| No target pose | Check real camera frames, nonzero calibrated intrinsics, matching resolution, `qr_size`, `target_pattern` and timestamped TF; check `/vision/qr` to see whether the code is read at all. |
| IK rejects a target | Check radius, Z travel, angle limits, radial tool yaw, `base_link` coordinates and collision state. |

For a useful error report, capture the first launch error, the build summary, active hardware settings and controller-list output. Do not describe a physical position as measured merely because it appears in `/joint_states`.
