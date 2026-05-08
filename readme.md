# ROS1 PSDK 3.13 全量接入实施计划

## Summary

在 `psdk_ros1` 中实现一个面向 ROS Noetic / Ubuntu 20.04 的 catkin 功能包，直接链接本地 `Payload-SDK-3.13.0`，参考 `psdk_ros2-main` 的模块结构，但覆盖 PSDK 3.13 全部模块。

采用已确认的“混合方案”：

- 常用高频功能提供强类型 ROS1 `topic/service/action`，便于业务节点直接使用。
- PSDK 3.13 全量 API 提供统一 JSON 调用入口，保证所有函数都能从 ROS1 调用。
- 所有 PSDK 数据、回调事件、服务调用结果、状态变化都发布到 ROS topic，满足“对应数据也要通过话题发布出来”。

## Key Changes

### 1. ROS1 包结构

在 `psdk_ros1` 中完成单包方案：

- `package.xml`：ROS1 catkin format 2，依赖 `roscpp`、`message_generation`、`actionlib`、`actionlib_msgs`、`std_msgs`、`sensor_msgs`、`geometry_msgs`、`nav_msgs`、`std_srvs`、`tf`、`tf2_ros`、`image_transport`。
- `CMakeLists.txt`：生成 msg/srv/action，编译 PSDK ROS1 wrapper，链接本地 `Payload-SDK-3.13.0/psdk_lib/lib/<arch>/libpayloadsdk.a`。
- `src/` 和 `include/psdk_ros1/`：实现 core、module manager、各 PSDK 模块封装、JSON API dispatcher、topic publisher。
- `launch/`、`config/`、`README.md`：提供可运行启动配置和完整说明。

架构约定：

- 一个主节点：`psdk_ros1_node`
- 默认命名空间：`/psdk`
- 模块内部 C++ 类名按模块命名，例如 `CoreModule`、`TelemetryModule`、`CameraManagerModule`、`FlightControllerModule`。
- 所有 PSDK 返回码统一保留为 `int64 return_code`，成功条件为 `DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS`。

### 2. PSDK 初始化与平台适配

实现 ROS1 版本 core 初始化流程：

- 从 ROS 参数读取：
  - `app_name`
  - `app_id`
  - `app_key`
  - `app_license`
  - `developer_account`
  - `baudrate`
  - `link_config_file_path`
  - `alias`
  - `firmware_version`
  - `serial_number`
- 注册 PSDK 平台 handler：
  - OSAL
  - UART
  - USB bulk
  - Network
  - Socket
  - FileSystem
- 读取 PSDK link config，支持：
  - UART only
  - UART + USB bulk
  - UART + network
  - USB bulk only
  - network only
- 调用：
  - `DjiCore_Init`
  - `DjiAircraftInfo_GetBaseInfo`
  - `DjiCore_SetAlias`
  - `DjiCore_SetFirmwareVersion`
  - `DjiCore_SetSerialNumber`
  - `DjiCore_ApplicationStart`
  - shutdown 时 `DjiCore_DeInit`

发布 core 状态：

- `/psdk/core/state`
- `/psdk/core/return_code`
- `/psdk/aircraft/base_info`
- `/psdk/aircraft/mobile_app_info`
- `/psdk/aircraft/connection_status`
- `/psdk/aircraft/version`

### 3. 强类型 ROS 接口

保留并迁移 `psdk_ros2-main/psdk_interfaces` 中已有接口到 ROS1：

- 遥测 msg：IMU、GPS、RTK、位置、速度、电池、ESC、飞行状态、控制权、云台状态、障碍物距离等。
- 相机 srv/action：拍照、录像、曝光、ISO、快门、光圈、对焦、变焦、文件列表、下载、删除、格式化 SD 卡。
- 飞控 srv/topic：起飞、降落、返航、控制权、摇杆控制、避障开关、home 点、高度设置。
- 云台 srv/topic：设置模式、复位、旋转控制、状态发布。
- 感知 srv/topic：双目参数、雷达、激光雷达、感知图像。
- HMS topic：HMS 消息表、错误码、错误等级、地面/空中提示文本。
- Liveview topic：H264 raw、解码后的 `sensor_msgs/Image`、相机源状态。

新增 ROS1 通用接口：

- `PsdkApi.srv`
  - request：`string module`、`string function`、`string request_json`
  - response：`bool success`、`int64 return_code`、`string response_json`、`string message`
- `PsdkApiResult.msg`
  - 每次 JSON API 调用后发布，包含 module/function/request/response/return_code/timestamp。
- `PsdkEvent.msg`
  - 所有 PSDK callback 事件统一发布，包含 module/event_name/payload_json/return_code/timestamp。
- `PsdkRawData.msg`
  - 二进制数据统一发布，包含 channel/source/metadata_json/uint8[] data。
- `PsdkModuleState.msg`
  - 各模块 init/deinit/available/error 状态。
- `PsdkJson.msg`
  - 用于发布结构暂不适合强类型化的数据。

所有强类型 service 的响应也同步发布到对应 topic，例如：

- `/psdk/camera/get_type/result`
- `/psdk/camera/storage_info`
- `/psdk/flight_controller/go_home_altitude`
- `/psdk/gimbal/command_result`
- `/psdk/perception/stereo_camera_parameters`

### 4. PSDK 3.13 模块覆盖

按 PSDK 头文件逐模块接入，最低要求是：模块可初始化/反初始化，公开可调用函数可通过 JSON service 调用，回调和返回数据发布 topic。

覆盖模块：

- Core / Platform / Logger
- Aircraft Info
- FC Subscription / Telemetry
- Flight Controller
- Camera Manager
- Payload Camera
- Gimbal Manager
- Payload Gimbal
- Liveview
- OpenAR
- HMS Manager
- HMS Customization
- Perception
- High Speed Data Channel
- Low Speed Data Channel
- MOP Channel
- Cloud API WebSocket
- Widget
- Widget Manager / Speaker
- Waypoint V2
- Waypoint V3
- Interest Point
- Positioning
- Time Sync
- Power Management
- Tethered Battery
- Upgrade
- XPort

实现规则：

- 对 ROS2 包已有模块，优先迁移其成熟逻辑到 ROS1。
- 对 ROS2 包未覆盖模块，先提供 JSON API + topic result，再为重要数据补强类型 msg。
- 对需要用户回调 handler 的 PSDK 功能，例如 payload camera、payload gimbal、widget、upgrade，提供默认可运行 handler，并把收到的请求/状态发布 topic。
- 对文件、视频、MOP、low/high speed data 等二进制数据，统一发布 `PsdkRawData`。
- 对无法安全自动执行的动作，例如电机、起飞、强制降落、升级、FTS、电源输出，保留接口，但默认要求参数 `safety_confirm: true` 或 ROS 参数启用。

### 5. Topic 发布规范

所有模块都遵守统一发布策略：

- 周期性数据：由参数控制频率，例如 `data_frequency.imu`、`data_frequency.gps`。
- 回调数据：收到 PSDK callback 后立即发布。
- service/action 结果：调用完成后发布一次 result topic。
- 原始数据：发布 `PsdkRawData`，metadata 放 JSON。
- 模块状态：init/deinit/error 时发布 `PsdkModuleState`。

默认 topic 分组：

- `/psdk/core/*`
- `/psdk/aircraft/*`
- `/psdk/telemetry/*`
- `/psdk/flight_controller/*`
- `/psdk/camera/*`
- `/psdk/payload_camera/*`
- `/psdk/gimbal/*`
- `/psdk/liveview/*`
- `/psdk/perception/*`
- `/psdk/hms/*`
- `/psdk/data_channel/*`
- `/psdk/waypoint_v2/*`
- `/psdk/waypoint_v3/*`
- `/psdk/widget/*`
- `/psdk/power/*`
- `/psdk/upgrade/*`
- `/psdk/xport/*`
- `/psdk/events`
- `/psdk/api/result`
- `/psdk/raw`

### 6. JSON API Dispatcher

实现一个中央 dispatcher：

- 输入 `module + function + request_json`
- 按白名单查找 PSDK 函数映射
- 解析 JSON 参数
- 调用对应 PSDK API
- 把输出结构编码为 JSON
- 返回 service response
- 同步发布 `/psdk/api/result`

JSON 规则：

- 枚举值统一用整数，README 中列出对应 PSDK enum 名称。
- 二进制输入支持 base64 字符串或 `uint8[]` 专用 service。
- 文件路径必须限制在参数 `allowed_file_roots` 下，防止误读写系统路径。
- callback 注册类 API 不直接暴露裸函数指针，改为启用/停用对应 ROS topic 桥接。

### 7. README 和文档

编写详细 `README.md`，必须包含：

- 项目定位：ROS1 wrapper for DJI PSDK 3.13
- 支持环境：ROS Noetic + Ubuntu 20.04
- 目录说明
- 依赖安装
- 编译步骤
- PSDK app 信息配置
- link config 配置
- launch 使用
- 参数表
- topic/service/action 总表
- JSON API 调用格式
- 每个 PSDK 模块的接入状态矩阵
- 飞控安全注意事项
- 相机/视频/文件下载说明
- 数据通道说明
- 常见报错排查
- 与 `psdk_ros2-main` 的差异说明
- 后续扩展强类型接口的方法

另建 `docs/psdk_3_13_api_matrix.md`：

- 按头文件列出 PSDK 3.13 函数
- 标明 ROS1 接入方式：typed service、typed topic、JSON API、event topic、raw topic
- 标明是否需要硬件/实机验证

## Test Plan

### 静态检查

- `catkin_lint psdk_ros1`
- 检查 `package.xml` 依赖闭环。
- 检查 `CMakeLists.txt` 是否能找到本地 PSDK include/lib。
- `rg "^T_DjiReturnCode Dji" Payload-SDK-3.13.0/psdk_lib/include` 与 `docs/psdk_3_13_api_matrix.md` 对照，确认无模块遗漏。
- 检查所有 msg/srv/action 能被 ROS1 message generation 解析。

### 编译验证

在 Ubuntu 20.04 + ROS Noetic：

- `catkin_make`
- `catkin_make install`
- 验证生成：
  - `psdk_ros1_node`
  - 所有 msg/srv/action Python/C++ 绑定
  - launch/config 安装路径正确

### 节点启动验证

无飞机硬件时：

- launch 能启动节点并报告缺少硬件或认证失败，不崩溃。
- `/psdk/core/state` 发布 error 状态。
- `/psdk/api/result` 能发布失败调用结果。

有 PSDK 硬件时：

- core 初始化成功。
- aircraft info topic 正常。
- telemetry 按频率发布。
- camera/gimbal/flight_controller/hms/liveview/perception 至少完成 init/deinit。
- JSON API 能调用每个模块的至少一个只读函数。
- callback 类模块能把事件发布到 `/psdk/events` 或对应模块 topic。

### 安全场景

- 未启用 safety 参数时，起飞、降落、强制降落、电机、电源、升级类危险接口返回拒绝并发布 result。
- 参数启用后，接口调用结果仍发布 topic。
- PSDK 返回错误码时，service response 和 topic result 必须同时包含错误码和可读 message。

## Assumptions

- 目标环境固定为 ROS Noetic / Ubuntu 20.04。
- 接口方案固定为混合方案：强类型常用接口 + 全量 JSON API。
- `Payload-SDK-3.13.0` 保持在仓库同级目录，不从网络下载 PSDK。
- 当前 Windows 工作区只做代码编写和静态检查；最终编译/实机验证在 Linux ROS1 环境完成。
- “所有功能接入 ROS1”定义为：每个 PSDK 3.13 模块和公开可调用 API 都至少有 ROS1 调用路径；关键常用功能有强类型接口。
- “对应数据通过话题发布”定义为：周期数据、callback 数据、raw 数据、service/action 调用结果、模块状态都必须发布到 topic。
