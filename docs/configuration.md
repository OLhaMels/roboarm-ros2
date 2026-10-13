# Configuration reference

[Documentation home](../README.md) · [Setup and operation](setup-and-operation.md) · [Architecture](architecture.md)

This reference describes the settings the current code actually reads. Examples are not measured parameters for your arm.

## Files and when they apply

| File | Role | Apply changes |
|---|---|---|
| `~/.config/roboarm/hardware.yaml` | Default writable geometry, limits, calibration and GPIO configuration | Save from the webpage while idle, then restart bringup |
| File passed with `hardware_config:=...` | Alternative hardware configuration; also the file edited by that launch's webpage | Restart using the same path |
| `src/roboarm_bringup/config/hardware.yaml` | Factory example copied on first default launch | Does not replace an existing user configuration |
| `src/roboarm_bringup/config/perception.yaml` | Default camera and marker parameters | Restart; prefer a writable copy selected with `perception_config` |
| `src/roboarm_bringup/config/controllers.yaml` | Controller types, joints and tracking tolerances | Ensure changes reach the installed package, then restart |
| `src/roboarm_bringup/config/moveit.yaml` | Planner, IK plugin and action-controller mapping | Ensure changes reach the installed package, then restart |
| `src/roboarm_bringup/urdf/roboarm.urdf.xacro` | Robot geometry and ros2_control interface declaration | Rebuild/install as needed, then restart |
| `src/roboarm_bringup/config/robot.srdf` | Planning groups and allowed self-collision pairs | Rebuild/install as needed, then restart |

These source paths are relative to the repository root on the Pi. The launched configuration files are found in the installed `roboarm_bringup` package. Use `ros2 pkg prefix roboarm_bringup` to identify that installation.

At launch, the hardware YAML supplies the URDF's position limits and initial positions. Its speed and acceleration values override the arm's MoveIt joint-limit entries. The task manager and webpage each retain a snapshot of those settings. The GPIO plugin receives them through the expanded URDF. Editing the YAML while nodes run does not change those snapshots.

There is no live limit-update service. Restarting keeps the planner, hardware and UI on one set of limits. The webpage's save operation also clears `reference_confirmed` in the saved file.

## Launch arguments

```bash
ros2 launch roboarm_bringup bringup.launch.py --show-args
```

| Argument | Default | Meaning |
|---|---|---|
| `hardware_mode` | `mock` | `mock` loads simulated arm hardware; `gpio` loads the A4988 plugin. No other values are accepted. |
| `hardware_config` | Empty | Empty uses `~/.config/roboarm/hardware.yaml`, creating it from the example if absent. Otherwise pass an absolute YAML path. |
| `reference_confirmed` | `false` | GPIO startup requires `true` on each launch after manually establishing initial coordinates. It does not command homing. |
| `demo_perception` | `true` | Synthetic images and target when true; USB camera and marker detection when false. GPIO mode rejects true. |
| `perception_config` | Installed example YAML | Camera and vision ROS parameters. Pass an absolute path for your calibration file. |
| `video_stream` | `false` | Also start `stream_node`: H.264 over TCP of the QR debug image. |
| `web_host` | `127.0.0.1` | Bind address for the webpage. `0.0.0.0` listens on network interfaces. Prefer loopback with an SSH tunnel. |

The Web UI has no authentication. Do not expose it to untrusted networks. Its port is a node parameter defaulting to `5000`; it is **not** a launch argument in this launch file. There are no current launch arguments for enabling RViz, disabling the camera, changing joint count, or choosing physical gripper hardware.

Example with separate settings:

```bash
ros2 launch roboarm_bringup bringup.launch.py \
  hardware_mode:=mock \
  hardware_config:=$HOME/.config/roboarm/my-arm.yaml
```

That custom file must already exist and contain a complete valid configuration.

## Coordinate conventions

The root frame is `base_link`; the tool frame is `tool0`. The arm has a vertical prismatic joint followed by rotation around +Z. At angle zero, the arm points along +X. Positive rotation turns +X toward +Y. Positive Z raises the carriage.

| Value | Unit |
|---|---|
| Z coordinates, Z bounds, `arm_radius`, `base_height` | Metres |
| Rotation coordinates and bounds in YAML or ROS | Radians |
| Rotation fields in the webpage's Motion and Pick/place forms | Degrees; converted before sending to ROS |
| Rotation fields in the webpage's Configuration section | Radians; stored directly |
| Velocity | Metres/second for Z; radians/second for rotation |
| Acceleration | Metres/second² for Z; radians/second² for rotation |
| Simulated gripper displacement | Metres |

`base_height` is the tool's height in `base_link` at Z joint coordinate zero. Tool height is therefore `base_height + z_joint`, not simply `z_joint`.

The tool's position and yaw are constrained by the mechanism:

```text
x = arm_radius * cos(rotation_joint)
y = arm_radius * sin(rotation_joint)
z_tool = base_height + z_joint
yaw_tool = rotation_joint
```

The IK plugin accepts a radial-position discrepancy of up to 1 mm and orientation discrepancy of up to 0.01 rad, then checks joint bounds and the requested collision callback. This tolerance is not an extra controllable radial axis.

## Top-level hardware fields

| Field | Meaning / accepted values |
|---|---|
| `commissioned` | Boolean acknowledgement of wiring/calibration readiness. Must be true for GPIO startup; may remain false for mock mode. |
| `reference_confirmed` | Boolean required in the YAML schema, but not trusted as startup permission. Launch uses its own argument. Keep false on disk. |
| `gpio_chip` | Absolute GPIO character-device path starting with `/dev/gpiochip`. Discover the correct RP1 device on the actual Pi. |
| `watchdog_seconds` | 0.05–1.0 seconds; default 0.25. If hardware writes stop arriving, the pulse worker latches a fault and disables drivers. |
| `arm_radius` | Measured distance from rotation axis to tool origin, in metres. Current validator accepts 0.1–2.0. |
| `base_height` | Measured height of tool origin at Z zero, in metres. Current validator accepts 0–2.0. |
| `joints` | Must contain exactly `z_joint` and `rotation_joint`. The gripper is configured separately as mock hardware in the URDF. |

## Per-joint fields

Both entries under `joints` require the following fields:

| Field | Meaning / validation |
|---|---|
| `lower`, `upper` | Ordered coordinate limits: `lower < upper`. Z limits must be inside ±2 m and rotation limits inside ±2π rad. These validation ranges are not mechanical limits for your robot. |
| `initial_position` | Actual coordinate manually established before launch. Must lie inside the configured limits. No motion is generated to find this position. |
| `steps_per_unit` | Positive number of full motor steps per metre or per radian, including the transmission ratio. |
| `max_step_rate` | Driver pulse-rate ceiling, 1–500 pulses/s; example value 400. |
| `max_velocity` | Positive joint-speed limit; must be at most `0.8 * max_step_rate / steps_per_unit`. |
| `max_acceleration` | Positive acceleration limit; must be at most `600 / steps_per_unit`, consistent with the driver's conservative ramp. |
| `direction_inverted` | Boolean that flips the DIR level corresponding to positive motion. |
| `direction_pin` | BCM offset connected to A4988 DIR. |
| `step_pin` | BCM offset connected to A4988 STEP. |
| `ms1_pin`, `ms2_pin`, `ms3_pin` | BCM offsets for microstep selection. This implementation drives all three LOW, selecting full steps. |
| `enable_pin` | BCM offset for active-low A4988 ENABLE. Required even though the original Python test did not configure one. |
| `min_limit_pin`, `max_limit_pin` | Optional switch inputs. Use `-1` for the current arm without switches. If fitted, the code expects normally closed contacts to ground and external 3.3 V pull-ups. |

Pin values must be integers from `-1` through `27`. Every assigned GPIO must be unique across both joints and all signals. In GPIO mode, `-1` is allowed only for optional limit inputs. The validator checks numbering and duplicates, not whether a chosen pin is available, exposed on your header or already used by another function. Confirm those physically.

All numeric settings must be finite; strings, booleans in numeric fields, NaN and infinity are rejected. The GPIO driver latches faults on invalid targets and driver errors rather than silently ignoring them.

## Calibration examples

For a screw-driven Z axis:

```text
steps_per_metre = steps_per_motor_revolution * gear_ratio / screw_lead_metres
```

For a belt axis, use the belt travel per output revolution in place of screw lead. `gear_ratio` means motor revolutions per output revolution.

A **hypothetical** 200-step motor directly driving an 8 mm-lead screw gives:

```text
200 / 0.008 = 25,000 steps/metre
1 step = 0.00004 metre = 0.04 mm
400 steps/second / 25,000 = 0.016 metre/second pulse capacity
Configuration velocity ceiling = 0.8 * 0.016 = 0.0128 metre/second
```

The example Z speed of 0.008 m/s lies below that capacity. The task manager additionally requests 50% velocity and acceleration scaling in MoveIt; a limit is not a promise that every motion reaches that speed.

For rotation:

```text
steps_per_radian = steps_per_motor_revolution * gear_ratio / (2*pi)
```

A **hypothetical** direct-drive 200-step motor gives about 31.831 steps/rad, or 1.8° per full step. With 10 motor revolutions per arm revolution, it gives about 318.31 steps/rad, or 0.18° per full step. Software does not create microsteps merely because a requested angle contains fractional degrees. Check that controller tolerances suit the real resolution.

After a carefully measured test, a candidate calibration correction is:

```text
new_steps_per_unit = old_steps_per_unit * requested_displacement / measured_displacement
```

Use displacements from the same initial reference and direction, not absolute coordinates. If the difference is caused by stalls, lost steps or backlash, fix that cause rather than hiding it in the scale. Recheck speed/acceleration capacity after changing the scale.

## Perception settings

Unlike hardware YAML, perception YAML follows the ROS node-parameter structure: a node name, then `ros__parameters`.

| Node / parameter | Meaning |
|---|---|
| `camera_node.backend` | `libcamera` (CSI camera via GStreamer libcamerasrc), `picamera2`, `v4l2` (USB, uses `device`) or `images` (loops files at `images_path`). Ignored in demo mode. |
| `camera_node.device` | OpenCV camera index for `v4l2`; example 0. |
| `camera_node.camera_name`, `gst_pipeline` | Optional libcamera camera id, or a complete pipeline ending in `appsink name=sink` producing RGB. |
| `camera_node.width`, `height` | Requested capture resolution; calibration must match the returned image dimensions. |
| `camera_node.fps` | Capture timer rate, greater than 0 and no more than 60. |
| `camera_node.camera_matrix` | Nine row-major intrinsics: `[fx, 0, cx, 0, fy, cy, 0, 0, 1]`. Use measured calibration. |
| `camera_node.distortion` | OpenCV distortion coefficients; normally `[k1, k2, p1, p2, k3]` for this plumb-bob setup. |
| `vision_node.qr_size` | Side of the printed QR symbol without its white margin, in metres; default 0.04. Must be greater than 0 and less than 1. |
| `vision_node.target_pattern` | Regular expression; only matching payloads become `/vision/target_pose`. Default accepts `2qi.us` and `*.ifurn.pro` links. |
| `vision_node.repeat_after` | Seconds a code must be out of view before it is reported again; default 5. |
| `vision_node.record_parts`, `parts_csv`, `fetch_timeout` | Follow scanned links to iFurn part pages and append rows to the CSV; default `~/.local/share/roboarm/parts.csv`. |
| `stream_node.image_topic`, `port`, `fps`, `bitrate` | Optional TCP video stream started with `video_stream:=true`; defaults `/vision/debug_image`, 8554, 15, 2000000. Needs `python3-av`. |
| `vision_node.target_frame` | Frame for published targets; keep `base_link` for compatibility with the task manager. |
| `vision_node.marker_to_tool_z` | Optional height offset from detected marker centre to tool target in base-frame Z; default 0.0. |

Do not add a conflicting `demo` value to the perception YAML: the launch file controls demonstration mode with `demo_perception`. Synthetic target geometry uses the configured arm radius and base height, with an example Z joint coordinate of 0.05 m; changing travel limits may make that example unreachable.

## Ownership of limits and faults

The task manager checks requested positions before submitting work. MoveIt plans using model and velocity/acceleration limits. The hardware plugin separately checks command bounds and estimates pulse positions. These checks all depend on a correct manual reference; they cannot detect that the physical arm slipped.

The 0.25-second default hardware watchdog observes controller-manager writes. It is **not** a browser-disconnect timeout. Closing the browser does not automatically cancel an accepted action. Cancel the goal before leaving it, or stop the system deliberately.

Driver deactivation or shutdown disables holding torque and invalidates the driver's reference. A GPIO fault is latched; there is no webpage fault-reset or homing service. Stop the stack, resolve the cause and manually reference before restarting. The task manager separately latches a fault when it cannot confirm downstream goal acceptance or cancellation.
