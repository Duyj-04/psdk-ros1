# psdk_ros1

`psdk_ros1` is a ROS Noetic wrapper for DJI Payload SDK 3.13. It is built as a single catkin package and links directly against the local `../Payload-SDK-3.13.0` tree.

The package follows a mixed interface design:

- Common workflows keep strong ROS1 msg/srv/action interfaces migrated from the ROS2 wrapper.
- Full PSDK 3.13 access is exposed through `/psdk/api/call` with JSON payloads.
- All service/API results, module states, callback events and raw data are published to ROS topics.

## Supported Environment

- Ubuntu 20.04
- ROS Noetic
- DJI Payload SDK 3.13 in `../Payload-SDK-3.13.0`
- Linux target architecture supported by DJI PSDK: x86_64, aarch64, armhf

This repository was edited on Windows, but final build and hardware validation must be done on Linux.

## Build

Place this package in a catkin workspace:

```bash
mkdir -p ~/catkin_ws/src
cp -r psdk_ros1 ~/catkin_ws/src/
cp -r Payload-SDK-3.13.0 ~/catkin_ws/
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

If PSDK is in another location:

```bash
catkin_make -DPSDK_PATH=/absolute/path/to/Payload-SDK-3.13.0
```

## Configuration

Edit `config/psdk_ros1.yaml`:

- `app_name`, `app_id`, `app_key`, `app_license`, `developer_account`, `baudrate`: DJI developer application credentials.
- `link_config_file_path`: optional PSDK link config file. If empty, the node registers UART, USB bulk and network handlers so PSDK can select the active link.
- `alias`, `serial_number`, `firmware_version`: values passed to PSDK core.
- `auto_start_core`: initialize PSDK core during node startup.
- `auto_start_modules`: initialize default PSDK modules after core startup.
- `allow_dangerous_commands`: global safety switch for motor, takeoff, force landing, format, delete, power output and upgrade operations.

Launch:

```bash
roslaunch psdk_ros1 psdk_ros1.launch
```

## Main Services

- `/psdk/core/initialize` (`std_srvs/Trigger`)
- `/psdk/core/shutdown` (`std_srvs/Trigger`)
- `/psdk/api/call` (`psdk_ros1/PsdkApi`)
- `/psdk/camera/format_sd_card` (`psdk_ros1/CameraFormatSdCard`)
- `/psdk/camera/get_type` (`psdk_ros1/CameraGetType`)
- `/psdk/camera/record_video` (`psdk_ros1/CameraRecordVideo`)
- `/psdk/camera/setup_streaming` (`psdk_ros1/CameraSetupStreaming`, starts/stops stream without publishing raw frames)
- `/psdk/camera/shoot_single_photo` (`psdk_ros1/CameraShootSinglePhoto`)
- `/psdk/gimbal/set_mode` (`psdk_ros1/GimbalSetMode`)
- `/psdk/gimbal/reset` (`psdk_ros1/GimbalReset`)
- `/psdk/flight_controller/set_go_home_altitude` (`psdk_ros1/SetGoHomeAltitude`)
- `/psdk/flight_controller/get_go_home_altitude` (`psdk_ros1/GetGoHomeAltitude`)

Example:

```bash
rosservice call /psdk/api/call "module: 'aircraft_info'
function: 'get_base_info'
request_json: '{}'"
```

Camera example:

```bash
rosservice call /psdk/api/call "module: 'camera_manager'
function: 'get_storage_info'
request_json: '{\"payload_index\":1}'"
```

Liveview stream data is not published by default. Enable raw H264 publication only through an explicit service/API request:

```bash
rosservice call /psdk/api/call "module: 'liveview'
function: 'start_h264_stream'
request_json: '{\"camera_position\":0,\"camera_source\":0,\"publish_raw\":true}'"
```

Stop it with:

```bash
rosservice call /psdk/api/call "module: 'liveview'
function: 'stop_h264_stream'
request_json: '{\"camera_position\":0,\"camera_source\":0}'"
```

Safety-gated flight example:

```bash
rosservice call /psdk/api/call "module: 'flight_controller'
function: 'takeoff'
request_json: '{\"safety_confirm\":true}'"
```

## Main Topics

- `/psdk/core/state` (`std_msgs/String`)
- `/psdk/core/return_code` (`std_msgs/Int64`)
- `/psdk/module_state` (`psdk_ros1/PsdkModuleState`)
- `/psdk/events` (`psdk_ros1/PsdkEvent`)
- `/psdk/raw` (`psdk_ros1/PsdkRawData`) for explicitly enabled raw streams and data channels
- `/psdk/api/result` (`psdk_ros1/PsdkApiResult`)
- `/psdk/json` (`psdk_ros1/PsdkJson`)

All JSON API responses are also published to `/psdk/api/result`. High-bandwidth image/video data is never published automatically.

## JSON API Format

Request:

```yaml
module: "camera_manager"
function: "get_camera_type"
request_json: "{\"payload_index\":1}"
```

Response:

```yaml
success: true
return_code: 0
response_json: "{\"camera_type\":42}"
message: "camera_manager get_camera_type"
```

Enums are passed as integer values matching DJI PSDK enum definitions. Complex callback registration APIs are bridged as ROS events/raw topics rather than exposing C function pointers.

## Implemented Dispatcher Groups

- `core`: init, deinit, application_start
- `aircraft_info`: base info, mobile app info, connection status, aircraft version
- lifecycle `init`/`deinit`: fc subscription, camera manager, gimbal manager, payload gimbal, liveview, HMS, perception, data channel, waypoint, interest point, positioning, time sync, power, tethered battery, widget, xport, payload camera, flight controller
- `camera_manager`: type, mode, photo, record, exposure, ISO, aperture, shutter, focus, optical/infrared zoom, storage, laser ranging, download/delete
- `flight_controller`: home, go-home altitude, joystick authority, joystick command, go-home, takeoff/landing/motor commands, obstacle avoidance
- `gimbal_manager`: mode, reset, rotate, restore factory settings
- `liveview`: start/stop H264, intraframe, unregister encoder callback. Raw H264 publication requires `"publish_raw":true`.
- `perception`: stereo parameters and unsubscribe operations
- data channel: high/low speed channel state and bandwidth, cloud API text send
- waypoint/interest point: lifecycle, start/stop/pause/resume/speed/action
- power/HMS/tether/widget/xport utility calls

See `docs/psdk_3_13_api_matrix.md` for the module coverage matrix.

## Strongly Typed Interfaces

The package keeps the ROS2 wrapper's common interfaces as ROS1 msg/srv/action definitions, including camera, gimbal, flight controller, HMS, perception and telemetry message types. The current node exposes the full working path through `/psdk/api/call`; typed services can be bound to the same dispatcher incrementally without changing PSDK logic.

## Safety

Dangerous commands are blocked unless one of these is true:

- `allow_dangerous_commands: true`
- The JSON request contains `"safety_confirm": true`

This applies to takeoff, landing, force landing, motor control, emergency brake, storage format, delete media, power output, gimbal factory restore and upgrade-class operations.

## Gimbal Topic Control and M300 Planner Bridge

Gimbal control is topic-first:

- `/psdk/gimbal/rotation_cmd` (`psdk_ros1/GimbalRotation`)
- `/psdk/gimbal/mode_cmd` (`psdk_ros1/GimbalMode`)
- `/psdk/gimbal/reset_cmd` (`psdk_ros1/GimbalResetCmd`)
- `/psdk/gimbal/command_result` (`psdk_ros1/PsdkApiResult`)
- `/psdk/gimbal/angles` (`psdk_ros1/GimbalAngles`)
- `/psdk/gimbal/status` (`psdk_ros1/GimbalStatus`)

`GimbalRotation.pitch`, `roll` and `yaw` are ROS-facing radians. The node converts them to degrees before calling `DjiGimbalManager_Rotate()`.

`/psdk/gimbal/angles` publishes DJI FC subscription gimbal attitude in a ground/NED-style frame, not raw H20 encoder joint angles. PSDK 3.13 does not expose a direct H20 three-axis encoder getter through `DjiGimbalManager`; use `/psdk/gimbal/status` limit flags for protection, or create a software-zero relative angle after a reset if your application needs "center equals zero" semantics.

## H20 Visual Target Tracker

The optional tracker node closes the loop from object detections to gimbal speed commands and can assist with aircraft yaw/body-frame lateral velocity near gimbal limits.

```bash
roslaunch psdk_ros1 target_tracker.launch
rosservice call /psdk/tracker/enable "data: true"
```

Inputs:

- `/detector/h20_wide/detections` (`vision_msgs/Detection2DArray`)
- `/detector/h20_zoom/detections` (`vision_msgs/Detection2DArray`)
- `/psdk/gimbal/angles`
- `/psdk/gimbal/status`

Outputs:

- `/psdk/gimbal/rotation_cmd`
- `/psdk/planner/control_mode`
- `/psdk/planner/enable`
- `/psdk/planner/velocity_cmd`
- `/psdk/tracker/state` (`psdk_ros1/TrackerState`)

Configure it through `config/target_tracker.yaml`. `assist_mode` supports `disabled`, `yaw_only` and `yaw_xy`; `yaw_xy` is intentionally speed- and acceleration-limited and should be validated with propellers removed before flight.
Optical zoom adjustment is available through `enable_zoom_control` and is off by default.

When using aircraft assist, make sure the PSDK flight controller module is initialized in your deployment and watch `/psdk/planner/command_result`; the tracker only publishes planner topics and does not bypass joystick authority handling.

Online planner bridge topics:

- `/psdk/planner/velocity_cmd` (`geometry_msgs/TwistStamped`)
- `/psdk/planner/pose_cmd` (`geometry_msgs/PoseStamped`)
- `/psdk/planner/enable` (`std_msgs/Bool`)
- `/psdk/planner/emergency_stop` (`std_msgs/Bool`)
- `/psdk/planner/control_mode` (`psdk_ros1/PlannerControlMode`)
- `/psdk/planner/state` (`psdk_ros1/PlannerState`)
- `/psdk/flight_controller/joystick_cmd_echo` (`psdk_ros1/JoystickCommand`)

The default online mode maps planner velocity commands to PSDK joystick body-frame horizontal velocity, vertical velocity and yaw-rate control. This matches common outputs from MARS SUPER planner, mission planner style bridges and FAST-Lab EGO-Planner adapters. The node does not implement planning; it only bridges planner outputs to PSDK.

Discrete mission topics:

- `/psdk/mission/path` (`nav_msgs/Path`)
- `/psdk/mission/kmz_file` (`psdk_ros1/KmzFile`)
- `/psdk/mission/kmz_path` (`std_msgs/String`)
- `/psdk/mission/command` (`psdk_ros1/MissionCommand`)
- `/psdk/mission/state` (`psdk_ros1/MissionState`)
- `/psdk/mission/event` (`psdk_ros1/PsdkEvent`)

Path missions require `mission.reference_latitude`, `mission.reference_longitude` and `mission.reference_altitude`. KMZ path loading is restricted by `mission.allow_kmz_file_path_roots`.

## Troubleshooting

- `DjiCore_Init failed`: verify app credentials, baudrate, physical link and DJI Pilot authorization.
- `PSDK library not found`: pass `-DPSDK_PATH=/path/to/Payload-SDK-3.13.0`.
- Node starts but modules fail: some modules require specific aircraft models, payload ports or Pilot app state.
- Services return unsupported: confirm `module` and `function` names match the dispatcher names in this README.
- No hardware attached: the node should still start, publish error states and return PSDK error codes through `/psdk/api/result`.

## Difference From psdk_ros2-main

`psdk_ros2-main` provides a ROS2 lifecycle wrapper for common modules. This package targets ROS1 and adds a full PSDK 3.13 JSON dispatcher so every module has a ROS1 access path, while preserving the strong interface files for common workflows.
