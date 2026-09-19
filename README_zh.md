# psdk_ros1 中文说明

`psdk_ros1` 是一个面向 ROS1 Noetic / Ubuntu 20.04 的 DJI Payload SDK 3.13 功能包。它直接链接本地 `../Payload-SDK-3.13.0`，参考 `psdk_ros2-main` 的常用模块接口，同时增加统一 JSON API 调用入口，用于覆盖 PSDK 3.13 的完整模块能力。

本包采用“混合接口”设计：

- 常用功能保留强类型 ROS1 `msg/srv/action`，方便业务节点直接调用。
- PSDK 3.13 全量模块通过 `/psdk/api/call` 统一 JSON service 暴露。
- 服务调用结果、模块状态、事件、低带宽状态数据会发布到 ROS topic。
- 图像/视频流这类高带宽数据不会默认发布，必须显式请求开启。

## 运行环境

推荐环境：

- Ubuntu 20.04
- ROS Noetic
- DJI Payload SDK 3.13
- 支持的 PSDK 目标架构：`x86_64`、`aarch64`、`armhf`

目录默认假设：

```text
catkin_ws/
  Payload-SDK-3.13.0/
  src/
    psdk_ros1/
```

如果 PSDK 放在其他位置，编译时用 `-DPSDK_PATH=/path/to/Payload-SDK-3.13.0` 指定。

## 编译

```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

指定 PSDK 路径：

```bash
catkin_make -DPSDK_PATH=/absolute/path/to/Payload-SDK-3.13.0
```

## 配置

配置文件位于：

```text
psdk_ros1/config/psdk_ros1.yaml
```

主要参数：

- `app_name`：DJI 开发者应用名称。
- `app_id`：DJI 应用 ID。
- `app_key`：DJI 应用 key。
- `app_license`：DJI 应用 license。
- `developer_account`：开发者账号。
- `baudrate`：串口波特率，默认 `460800`。
- `link_config_file_path`：PSDK link config 路径，可为空。
- `alias`：PSDK 应用别名。
- `serial_number`：负载设备序列号。
- `firmware_version`：固件版本数组，例如 `[1, 0, 0, 0]`。
- `auto_start_core`：节点启动时是否自动初始化 PSDK core。
- `auto_start_modules`：core 初始化后是否自动初始化默认模块。
- `allow_dangerous_commands`：是否全局允许危险操作，默认 `false`。

危险操作包括：电机控制、起飞、降落、强制降落、格式化存储、删除文件、高功率输出、云台恢复出厂设置、升级类操作等。

推荐保持：

```yaml
allow_dangerous_commands: false
```

需要执行危险操作时，在单次 JSON 请求中传入：

```json
{"safety_confirm": true}
```

## 启动

```bash
roslaunch psdk_ros1 psdk_ros1.launch
```

指定配置文件：

```bash
roslaunch psdk_ros1 psdk_ros1.launch config:=/path/to/psdk_ros1.yaml
```

## 主要服务

核心服务：

- `/psdk/core/initialize`：初始化 PSDK core。
- `/psdk/core/shutdown`：关闭 PSDK。
- `/psdk/api/call`：统一 JSON API 调用入口。

已绑定的常用强类型服务：

- `/psdk/camera/format_sd_card`
- `/psdk/camera/get_type`
- `/psdk/camera/record_video`
- `/psdk/camera/setup_streaming`
- `/psdk/camera/shoot_single_photo`
- `/psdk/gimbal/set_mode`
- `/psdk/gimbal/reset`
- `/psdk/flight_controller/set_go_home_altitude`
- `/psdk/flight_controller/get_go_home_altitude`

注意：`/psdk/camera/setup_streaming` 只负责启动或停止流，不会默认把图像流发布到 `/psdk/raw`。

## 主要话题

- `/psdk/core/state`：core 状态文本。
- `/psdk/core/return_code`：core 返回码。
- `/psdk/module_state`：模块初始化、反初始化、可用状态。
- `/psdk/events`：PSDK callback 或事件桥接。
- `/psdk/raw`：显式启用后的原始数据。
- `/psdk/api/result`：每次 `/psdk/api/call` 的调用结果。
- `/psdk/json`：结构暂不适合强类型化的数据。

所有 JSON API 调用结果都会同步发布到 `/psdk/api/result`。

## JSON API 使用方式

服务类型：

```text
psdk_ros1/PsdkApi
```

请求字段：

- `module`：模块名。
- `function`：函数名。
- `request_json`：JSON 字符串参数。

响应字段：

- `success`：是否成功。
- `return_code`：PSDK 返回码。
- `response_json`：JSON 字符串结果。
- `message`：可读说明。

查询飞机基础信息：

```bash
rosservice call /psdk/api/call "module: 'aircraft_info'
function: 'get_base_info'
request_json: '{}'"
```

查询相机存储信息：

```bash
rosservice call /psdk/api/call "module: 'camera_manager'
function: 'get_storage_info'
request_json: '{\"payload_index\":1}'"
```

查询云台或相机相关枚举时，JSON 中的枚举值直接使用 DJI PSDK 头文件中的整数值。

## 图像流和带宽

为了避免占满链路带宽，liveview 图像/视频流默认不会发布到 ROS topic。

如果确实需要把 H264 原始帧发布到 `/psdk/raw`，必须显式开启：

```bash
rosservice call /psdk/api/call "module: 'liveview'
function: 'start_h264_stream'
request_json: '{\"camera_position\":0,\"camera_source\":0,\"publish_raw\":true}'"
```

停止：

```bash
rosservice call /psdk/api/call "module: 'liveview'
function: 'stop_h264_stream'
request_json: '{\"camera_position\":0,\"camera_source\":0}'"
```

如果只想让 PSDK 侧启动流，但不把数据发布到 ROS topic：

```bash
rosservice call /psdk/api/call "module: 'liveview'
function: 'start_h264_stream'
request_json: '{\"camera_position\":0,\"camera_source\":0,\"publish_raw\":false}'"
```

## 已覆盖模块

当前 JSON dispatcher 覆盖以下模块入口：

- `core`
- `aircraft_info`
- `fc_subscription`
- `flight_controller`
- `camera_manager`
- `payload_camera`
- `gimbal_manager`
- `payload_gimbal`
- `liveview`
- `open_ar`
- `hms_manager`
- `hms_customization`
- `perception`
- `high_speed_data_channel`
- `low_speed_data_channel`
- `mop_channel`
- `cloud_api`
- `widget`
- `widget_manager`
- `waypoint_v2`
- `waypoint_v3`
- `interest_point`
- `positioning`
- `time_sync`
- `power_management`
- `tethered_battery`
- `xport`

更详细的模块矩阵见：

```text
psdk_ros1/docs/psdk_3_13_api_matrix.md
```

## 常用 JSON 函数示例

初始化模块：

```bash
rosservice call /psdk/api/call "module: 'camera_manager'
function: 'init'
request_json: '{}'"
```

设置云台模式：

```bash
rosservice call /psdk/api/call "module: 'gimbal_manager'
function: 'set_mode'
request_json: '{\"payload_index\":1,\"mode\":0}'"
```

云台旋转：

```bash
rosservice call /psdk/api/call "module: 'gimbal_manager'
function: 'rotate'
request_json: '{\"payload_index\":1,\"rotation_mode\":0,\"pitch_deg\":0,\"roll_deg\":0,\"yaw_deg\":10,\"time\":1.0}'"
```

获取返航高度：

```bash
rosservice call /psdk/api/call "module: 'flight_controller'
function: 'get_go_home_altitude'
request_json: '{}'"
```

设置返航高度：

```bash
rosservice call /psdk/api/call "module: 'flight_controller'
function: 'set_go_home_altitude'
request_json: '{\"altitude\":50}'"
```

起飞，危险操作，需要确认：

```bash
rosservice call /psdk/api/call "module: 'flight_controller'
function: 'takeoff'
request_json: '{\"safety_confirm\":true}'"
```

## 与 psdk_ros2-main 的区别

`psdk_ros2-main` 是 ROS2 lifecycle 风格的常用功能 wrapper，覆盖 telemetry、camera、gimbal、flight control、liveview、HMS、perception 等模块。

`psdk_ros1` 的目标是：

- 适配 ROS1 Noetic。
- 保留常用强类型接口文件。
- 增加全量 JSON API dispatcher。
- 让 PSDK 3.13 的每个模块至少有 ROS1 调用路径。
- 把返回结果、事件、状态、必要原始数据发布到 ROS topic。

## 注意事项

- 当前工作区是在 Windows 上编辑的，无法直接完成 ROS Noetic 编译验证。
- 最终需要在 Ubuntu 20.04 + ROS Noetic + DJI PSDK 实机环境中执行 `catkin_make`。
- 不同飞机、挂载口、负载设备、Pilot 版本支持的 PSDK 功能不同，某些 API 返回失败并不一定是 ROS 包错误。
- 高带宽数据不要默认发布到 ROS topic，应按需通过 service/API 开启。

## 故障排查

`DjiCore_Init failed`：

- 检查 app 信息是否正确。
- 检查串口、USB bulk、网口链路。
- 检查 DJI Pilot 是否完成授权。

找不到 `libpayloadsdk.a`：

- 检查 `Payload-SDK-3.13.0/psdk_lib/lib/<arch>/libpayloadsdk.a` 是否存在。
- 编译时指定 `-DPSDK_PATH=/absolute/path/to/Payload-SDK-3.13.0`。

模块初始化失败：

- 确认该模块是否被当前机型、挂载位置、负载设备支持。
- 查看 `/psdk/module_state` 和 `/psdk/api/result`。

图像流没有数据：

- 确认调用 `liveview/start_h264_stream` 时传入了 `"publish_raw": true`。
- 检查带宽、相机源、挂载位置和 Pilot 设置。

## 云台 topic 控制

云台控制推荐使用 topic，service 和 JSON API 继续保留用于一次性命令和调试。

订阅 topic：

- `/psdk/gimbal/rotation_cmd`：`psdk_ros1/GimbalRotation`
- `/psdk/gimbal/mode_cmd`：`psdk_ros1/GimbalMode`
- `/psdk/gimbal/reset_cmd`：`psdk_ros1/GimbalResetCmd`

发布 topic：

- `/psdk/gimbal/command_result`：`psdk_ros1/PsdkApiResult`

`GimbalRotation` 的 `pitch`、`roll`、`yaw` 单位是弧度，节点内部会转换成 PSDK 使用的角度制。

示例：

```bash
rostopic pub /psdk/gimbal/rotation_cmd psdk_ros1/GimbalRotation "header:
  stamp: now
payload_index: 1
rotation_mode: 0
pitch: -0.35
roll: 0.0
yaw: 0.52
time: 1.0"
```

## M300 在线规划控制

在线规划控制面向 SUPER planner、EGO-Planner、mission planner 等常见机器人规划器。`psdk_ros1` 不实现规划算法，只把规划器输出桥接到 PSDK joystick 控制。

订阅 topic：

- `/psdk/planner/velocity_cmd`：`geometry_msgs/TwistStamped`
- `/psdk/planner/pose_cmd`：`geometry_msgs/PoseStamped`
- `/psdk/planner/enable`：`std_msgs/Bool`
- `/psdk/planner/emergency_stop`：`std_msgs/Bool`
- `/psdk/planner/control_mode`：`psdk_ros1/PlannerControlMode`

发布 topic：

- `/psdk/planner/state`：`psdk_ros1/PlannerState`
- `/psdk/planner/command_result`：`psdk_ros1/PsdkApiResult`
- `/psdk/flight_controller/joystick_cmd_echo`：`psdk_ros1/JoystickCommand`

默认控制模式：

- 水平速度控制。
- 垂直速度控制。
- yaw rate 控制。
- 机体系 body/FRU。
- stable mode 开启。

启动在线控制：

```bash
rostopic pub /psdk/planner/enable std_msgs/Bool "data: true" -1
```

发布速度指令：

```bash
rostopic pub /psdk/planner/velocity_cmd geometry_msgs/TwistStamped "header:
  stamp: now
twist:
  linear:
    x: 0.5
    y: 0.0
    z: 0.0
  angular:
    z: 0.2"
```

停止在线控制并释放控制权：

```bash
rostopic pub /psdk/planner/enable std_msgs/Bool "data: false" -1
```

急停：

```bash
rostopic pub /psdk/planner/emergency_stop std_msgs/Bool "data: true" -1
```

如果超过 `planner.command_timeout` 没有收到新指令，节点会发送零速度 hold 指令，并在 `/psdk/planner/state` 中发布 timeout 状态。

## 离散航点和 KMZ 任务

离散航点和 KMZ 航线都通过 topic 输入。

Path 工作流：

- `/psdk/mission/path`：`nav_msgs/Path`
- `/psdk/mission/command`：`psdk_ros1/MissionCommand`
- `/psdk/mission/state`：`psdk_ros1/MissionState`
- `/psdk/mission/event`：`psdk_ros1/PsdkEvent`

KMZ 工作流：

- `/psdk/mission/kmz_file`：`psdk_ros1/KmzFile`
- `/psdk/mission/kmz_path`：`std_msgs/String`
- `/psdk/mission/command`：`psdk_ros1/MissionCommand`

Path 航点需要配置 ENU 到经纬度的参考原点：

```yaml
mission:
  reference_latitude: 22.0
  reference_longitude: 113.0
  reference_altitude: 10.0
```

KMZ 路径读取受 `mission.allow_kmz_file_path_roots` 限制，避免误读系统任意路径。

任务命令：

- `UPLOAD_PATH`
- `UPLOAD_KMZ`
- `START`
- `PAUSE`
- `RESUME`
- `STOP`
- `SET_CRUISE_SPEED`
- `CLEAR`
