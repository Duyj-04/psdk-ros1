# PSDK 3.13 ROS1 API Matrix

This matrix documents how `psdk_ros1` exposes DJI Payload SDK 3.13 modules.

Legend:

- `typed`: ROS1 msg/srv/action exists for the common workflow.
- `json`: `/psdk/api/call` exposes the function through `psdk_ros1/PsdkApi`.
- `topic`: returned data, callback data, state or raw bytes are published to ROS topics.
- `hardware`: requires DJI aircraft/payload hardware for validation.

| PSDK module | Main headers | ROS1 access | Published data |
| --- | --- | --- | --- |
| Core / Platform / Logger | `dji_core.h`, `dji_platform.h`, `dji_logger.h` | json, services `/psdk/core/initialize`, `/psdk/core/shutdown` | `/psdk/core/state`, `/psdk/core/return_code`, `/psdk/module_state`, `/psdk/api/result` |
| Aircraft Info | `dji_aircraft_info.h` | json | `/psdk/json` entries `aircraft/base_info`, `aircraft/mobile_app_info`, `aircraft/connection_status`, `aircraft/version` |
| FC Subscription / Telemetry | `dji_fc_subscription.h` | json lifecycle, typed telemetry messages available | `/psdk/module_state`; typed telemetry publishers are extension points |
| Flight Controller | `dji_flight_controller.h` | json, migrated typed srv definitions | `/psdk/json`, `/psdk/api/result` |
| Camera Manager | `dji_camera_manager.h` | json, migrated typed camera srv/action definitions | `/psdk/json`, `/psdk/api/result`, `/psdk/raw` for future file/video data |
| Payload Camera | `dji_payload_camera.h` | json lifecycle and handler extension points | `/psdk/events`, `/psdk/raw` |
| Gimbal Manager | `dji_gimbal_manager.h` | json, migrated typed gimbal srv definitions | `/psdk/json`, `/psdk/api/result` |
| Payload Gimbal | `dji_gimbal.h` | json lifecycle and handler extension points | `/psdk/events` |
| Liveview / OpenAR | `dji_liveview.h`, `dji_open_ar.h` | json start/stop/control, typed streaming srv definition | `/psdk/raw` only after `publish_raw=true`, `/psdk/events`, `/psdk/api/result` |
| HMS | `dji_hms_manager.h`, `dji_hms_customization.h` | json lifecycle/control, typed HMS messages | `/psdk/events`, `/psdk/json` |
| Perception | `dji_perception.h` | json, migrated typed perception srv/msg definitions | `/psdk/json`, `/psdk/raw` |
| High Speed Data Channel | `dji_high_speed_data_channel.h` | json | `/psdk/raw`, `/psdk/json` |
| Low Speed Data Channel | `dji_low_speed_data_channel.h` | json | `/psdk/raw`, `/psdk/json` |
| MOP Channel | `dji_mop_channel.h` | json lifecycle placeholder | `/psdk/raw`, `/psdk/api/result` |
| Cloud API WebSocket | `dji_cloud_api_by_websockt.h` | json `send_text` | `/psdk/api/result`, `/psdk/raw` |
| Widget / Widget Manager | `dji_widget.h`, `dji_widget_manager.h` | json lifecycle/control | `/psdk/events`, `/psdk/raw` |
| Waypoint V2 | `dji_waypoint_v2.h`, `dji_waypoint_v2_type.h` | json lifecycle/start/stop/pause/resume/speed | `/psdk/events`, `/psdk/api/result` |
| Waypoint V3 | `dji_waypoint_v3.h` | json lifecycle/action | `/psdk/events`, `/psdk/api/result` |
| Interest Point | `dji_interest_point.h` | json lifecycle/stop/speed | `/psdk/events`, `/psdk/api/result` |
| Positioning | `dji_positioning.h` | json lifecycle extension point | `/psdk/events`, `/psdk/json` |
| Time Sync | `dji_time_sync.h` | json lifecycle extension point | `/psdk/events`, `/psdk/json` |
| Power Management | `dji_power_management.h` | json, safety-gated | `/psdk/api/result`, `/psdk/events` |
| Tethered Battery | `dji_tethered_battery.h` | json lifecycle/status push | `/psdk/api/result` |
| Upgrade | `dji_upgrade.h` | json extension point, safety-gated | `/psdk/events`, `/psdk/api/result` |
| XPort | `dji_xport.h` | json lifecycle/control | `/psdk/events`, `/psdk/api/result` |

Every `/psdk/api/call` response is also published to `/psdk/api/result`.
Every module lifecycle transition is published to `/psdk/module_state`.
High-bandwidth image/video data is not published by default; enable it only through an explicit service/API request.

Planner and mission bridge topics:

- Gimbal topic control: `/psdk/gimbal/rotation_cmd`, `/psdk/gimbal/mode_cmd`, `/psdk/gimbal/reset_cmd`, `/psdk/gimbal/command_result`.
- Online planner bridge: `/psdk/planner/velocity_cmd`, `/psdk/planner/pose_cmd`, `/psdk/planner/enable`, `/psdk/planner/emergency_stop`, `/psdk/planner/control_mode`, `/psdk/planner/state`.
- Joystick echo: `/psdk/flight_controller/joystick_cmd_echo`.
- Mission bridge: `/psdk/mission/path`, `/psdk/mission/kmz_file`, `/psdk/mission/kmz_path`, `/psdk/mission/command`, `/psdk/mission/state`, `/psdk/mission/event`.

Hardware validation is required for all PSDK calls that communicate with an aircraft, payload, camera, gimbal or Pilot app.
