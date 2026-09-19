#include <psdk_ros1/psdk_ros1_wrapper.hpp>

#include <std_msgs/Int64.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <sstream>
#include <thread>

/**
 * This implementation is the runtime bridge between ROS1 and DJI PSDK 3.13.
 * ROS callbacks enter through services, subscribers, and timers; PSDK results
 * leave through the corresponding result/state/event topics.  The source keeps
 * the PSDK calls in module-specific dispatchers so the ROS-facing contract is
 * stable even when a module has no strongly typed service yet.
 */
namespace psdk_ros1
{
namespace
{
// These aliases keep dispatcher return handling readable.  Unsupported means
// "no branch matched"; a real DJI return code is still published to ROS.
const T_DjiReturnCode kSuccess = DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
const T_DjiReturnCode kUnsupported = DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
const double kRadToDeg = 180.0 / 3.14159265358979323846;
const double kDegToRad = 3.14159265358979323846 / 180.0;

// DJI invokes several callbacks as C functions, so these small global bridges
// carry only the ROS publishers/configuration needed by those callbacks.
ros::Publisher g_raw_pub;
ros::Publisher g_gimbal_angles_pub;
ros::Publisher g_gimbal_status_pub;
uint8_t g_gimbal_feedback_payload_index = 1;
bool g_liveview_raw_publish_enabled = false;

template <typename T>
std::string numberJson(const std::string& key, T value)
{
  std::ostringstream ss;
  ss << "{\"" << key << "\":" << value << "}";
  return ss.str();
}

// Liveview data is forwarded as raw bytes only when the API request explicitly
// enables raw publication; this avoids publishing a high-rate stream by default.
void liveviewH264Callback(E_DjiLiveViewCameraPosition position, const uint8_t* buf, uint32_t len)
{
  if (!g_liveview_raw_publish_enabled || !g_raw_pub) {
    return;
  }
  PsdkRawData msg;
  msg.header.stamp = ros::Time::now();
  msg.channel = "liveview";
  msg.source = "h264";
  std::ostringstream metadata;
  metadata << "{\"camera_position\":" << static_cast<int>(position) << ",\"len\":" << len << "}";
  msg.metadata_json = metadata.str();
  if (buf != nullptr && len > 0) {
    msg.data.assign(buf, buf + len);
  }
  g_raw_pub.publish(msg);
}

// FC subscription callbacks receive packed SDK structs.  Validate the buffer
// size before copying, then convert SDK angle units (degrees) to ROS radians.
T_DjiReturnCode gimbalAnglesCallback(const uint8_t* data, uint16_t data_size, const T_DjiDataTimestamp*)
{
  if (!g_gimbal_angles_pub || data == nullptr || data_size < sizeof(T_DjiFcSubscriptionGimbalAngles)) {
    return kSuccess;
  }

  T_DjiFcSubscriptionGimbalAngles angles = {};
  std::memcpy(&angles, data, sizeof(angles));
  GimbalAngles msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "dji_gimbal_ground_ned";
  msg.payload_index = g_gimbal_feedback_payload_index;
  msg.pitch = static_cast<float>(angles.x * kDegToRad);
  msg.roll = static_cast<float>(angles.y * kDegToRad);
  msg.yaw = static_cast<float>(angles.z * kDegToRad);
  g_gimbal_angles_pub.publish(msg);
  return kSuccess;
}

// Status fields are copied without changing their semantic values and are
// published on the gimbal/status topic by the wrapper-owned ROS publisher.
T_DjiReturnCode gimbalStatusCallback(const uint8_t* data, uint16_t data_size, const T_DjiDataTimestamp*)
{
  if (!g_gimbal_status_pub || data == nullptr || data_size < sizeof(T_DjiFcSubscriptionGimbalStatus)) {
    return kSuccess;
  }

  T_DjiFcSubscriptionGimbalStatus status = {};
  std::memcpy(&status, data, sizeof(status));
  GimbalStatus msg;
  msg.header.stamp = ros::Time::now();
  msg.mount_status = status.mountStatus;
  msg.is_busy = status.isBusy;
  msg.pitch_limited = status.pitchLimited;
  msg.roll_limited = status.rollLimited;
  msg.yaw_limited = status.yawLimited;
  msg.calibrating = status.calibrating;
  msg.prev_calibration_result = status.prevCalibrationgResult;
  msg.installed_direction = status.installedDirection;
  msg.disabled_mvo = status.disabled_mvo;
  msg.gear_show_unable = status.gear_show_unable;
  msg.gyro_falut = status.gyroFalut;
  msg.esc_pitch_status = status.escPitchStatus;
  msg.esc_roll_status = status.escRollStatus;
  msg.esc_yaw_status = status.escYawStatus;
  msg.drone_data_recv = status.droneDataRecv;
  msg.init_unfinished = status.initUnfinished;
  msg.fw_updating = status.FWUpdating;
  g_gimbal_status_pub.publish(msg);
  return kSuccess;
}

// Map user frequency settings to the finite topic frequencies supported by
// DjiFcSubscription.  Values between supported steps round down safely.
E_DjiDataSubscriptionTopicFreq normalizeSubscriptionFrequency(int frequency)
{
  if (frequency >= 50) {
    return DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ;
  }
  if (frequency >= 10) {
    return DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ;
  }
  if (frequency >= 5) {
    return DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ;
  }
  return DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ;
}
}  // namespace

// Construct the ROS surface first.  PSDK is not initialized here so the node
// can advertise state/error topics even when credentials or hardware are absent.
PsdkRos1Wrapper::PsdkRos1Wrapper(ros::NodeHandle nh, ros::NodeHandle private_nh)
  : nh_(nh), private_nh_(private_nh), platform_ready_(false), core_initialized_(false), modules_initialized_(false),
    gimbal_angles_feedback_subscribed_(false), gimbal_status_feedback_subscribed_(false),
    planner_enabled_(false), planner_has_authority_(false), planner_emergency_stop_(false),
    planner_has_velocity_cmd_(false), planner_has_pose_cmd_(false), mission_has_path_(false), mission_has_kmz_(false)
{
  // Latched state topics retain the last known state for late subscribers;
  // event/result topics remain non-latched streams of individual operations.
  core_state_pub_ = nh_.advertise<std_msgs::String>("core/state", 10, true);
  core_return_code_pub_ = nh_.advertise<std_msgs::Int64>("core/return_code", 10, true);
  module_state_pub_ = nh_.advertise<PsdkModuleState>("module_state", 50, true);
  event_pub_ = nh_.advertise<PsdkEvent>("events", 100);
  raw_pub_ = nh_.advertise<PsdkRawData>("raw", 20);
  g_raw_pub = raw_pub_;
  api_result_pub_ = nh_.advertise<PsdkApiResult>("api/result", 100);
  json_pub_ = nh_.advertise<PsdkJson>("json", 100);
  gimbal_command_result_pub_ = nh_.advertise<PsdkApiResult>("gimbal/command_result", 20);
  gimbal_angles_pub_ = nh_.advertise<GimbalAngles>("gimbal/angles", 20);
  gimbal_status_pub_ = nh_.advertise<GimbalStatus>("gimbal/status", 20);
  g_gimbal_angles_pub = gimbal_angles_pub_;
  g_gimbal_status_pub = gimbal_status_pub_;
  planner_state_pub_ = nh_.advertise<PlannerState>("planner/state", 20, true);
  planner_command_result_pub_ = nh_.advertise<PsdkApiResult>("planner/command_result", 50);
  joystick_cmd_echo_pub_ = nh_.advertise<JoystickCommand>("flight_controller/joystick_cmd_echo", 50);
  mission_state_pub_ = nh_.advertise<MissionState>("mission/state", 20, true);
  mission_event_pub_ = nh_.advertise<PsdkEvent>("mission/event", 50);

  // Command topics are translated to PSDK calls by the subscriber callbacks.
  gimbal_rotation_sub_ = nh_.subscribe("gimbal/rotation_cmd", 10, &PsdkRos1Wrapper::gimbalRotationCb, this);
  gimbal_mode_sub_ = nh_.subscribe("gimbal/mode_cmd", 10, &PsdkRos1Wrapper::gimbalModeCb, this);
  gimbal_reset_sub_ = nh_.subscribe("gimbal/reset_cmd", 10, &PsdkRos1Wrapper::gimbalResetCb, this);
  planner_velocity_sub_ = nh_.subscribe("planner/velocity_cmd", 10, &PsdkRos1Wrapper::plannerVelocityCb, this);
  planner_pose_sub_ = nh_.subscribe("planner/pose_cmd", 10, &PsdkRos1Wrapper::plannerPoseCb, this);
  planner_enable_sub_ = nh_.subscribe("planner/enable", 10, &PsdkRos1Wrapper::plannerEnableCb, this);
  planner_emergency_stop_sub_ = nh_.subscribe("planner/emergency_stop", 10, &PsdkRos1Wrapper::plannerEmergencyStopCb, this);
  planner_control_mode_sub_ = nh_.subscribe("planner/control_mode", 10, &PsdkRos1Wrapper::plannerControlModeCb, this);
  mission_path_sub_ = nh_.subscribe("mission/path", 1, &PsdkRos1Wrapper::missionPathCb, this);
  mission_command_sub_ = nh_.subscribe("mission/command", 10, &PsdkRos1Wrapper::missionCommandCb, this);
  mission_kmz_file_sub_ = nh_.subscribe("mission/kmz_file", 1, &PsdkRos1Wrapper::missionKmzFileCb, this);
  mission_kmz_path_sub_ = nh_.subscribe("mission/kmz_path", 1, &PsdkRos1Wrapper::missionKmzPathCb, this);

  // Services cover common operations directly; /api/call is the generic JSON
  // entry point used for modules without a dedicated typed service.
  initialize_srv_ = nh_.advertiseService("core/initialize", &PsdkRos1Wrapper::handleInitialize, this);
  shutdown_srv_ = nh_.advertiseService("core/shutdown", &PsdkRos1Wrapper::handleShutdown, this);
  api_srv_ = nh_.advertiseService("api/call", &PsdkRos1Wrapper::handleApi, this);
  camera_format_sd_card_srv_ = nh_.advertiseService("camera/format_sd_card", &PsdkRos1Wrapper::handleCameraFormatSdCard, this);
  camera_get_type_srv_ = nh_.advertiseService("camera/get_type", &PsdkRos1Wrapper::handleCameraGetType, this);
  camera_record_video_srv_ = nh_.advertiseService("camera/record_video", &PsdkRos1Wrapper::handleCameraRecordVideo, this);
  camera_setup_streaming_srv_ = nh_.advertiseService("camera/setup_streaming", &PsdkRos1Wrapper::handleCameraSetupStreaming, this);
  camera_shoot_single_photo_srv_ = nh_.advertiseService("camera/shoot_single_photo", &PsdkRos1Wrapper::handleCameraShootSinglePhoto, this);
  gimbal_set_mode_srv_ = nh_.advertiseService("gimbal/set_mode", &PsdkRos1Wrapper::handleGimbalSetMode, this);
  gimbal_reset_srv_ = nh_.advertiseService("gimbal/reset", &PsdkRos1Wrapper::handleGimbalReset, this);
  set_go_home_altitude_srv_ = nh_.advertiseService("flight_controller/set_go_home_altitude", &PsdkRos1Wrapper::handleSetGoHomeAltitude, this);
  get_go_home_altitude_srv_ = nh_.advertiseService("flight_controller/get_go_home_altitude", &PsdkRos1Wrapper::handleGetGoHomeAltitude, this);
  aircraft_info_timer_ = nh_.createTimer(ros::Duration(1.0), &PsdkRos1Wrapper::aircraftInfoTimerCb, this);
  planner_timer_ = nh_.createTimer(ros::Duration(1.0 / 30.0), &PsdkRos1Wrapper::plannerTimerCb, this, false, false);

  planner_control_mode_.horizontal_mode = PlannerControlMode::HORIZONTAL_VELOCITY;
  planner_control_mode_.vertical_mode = PlannerControlMode::VERTICAL_VELOCITY;
  planner_control_mode_.yaw_mode = PlannerControlMode::YAW_RATE;
  planner_control_mode_.frame = PlannerControlMode::FRAME_BODY;
  planner_control_mode_.stable_mode = true;
}

PsdkRos1Wrapper::~PsdkRos1Wrapper()
{
  shutdown();
}

// Forward startup order: parameters -> platform handlers -> core -> modules.
// Each stage publishes its own state so partial startup remains diagnosable.
bool PsdkRos1Wrapper::start()
{
  if (!loadParams()) {
    publishCoreState("parameter_error", kUnsupported, "Missing or invalid ROS parameters");
    return false;
  }

  if (!registerPlatformHandlers()) {
    publishCoreState("platform_error", kUnsupported, "Failed to register PSDK platform handlers");
    return false;
  }

  if (private_nh_.param("auto_start_core", true)) {
    if (!initCore()) {
      return false;
    }
    if (params_.auto_start_modules) {
      initDefaultModules();
    }
  } else {
    publishCoreState("ready_not_started", kSuccess, "Platform handlers are registered; call /psdk/core/initialize");
  }
  return true;
}

// Reverse shutdown order.  Module deinitialization precedes DjiCore_DeInit;
// repeated calls are harmless because the state flags gate each stage.
void PsdkRos1Wrapper::shutdown()
{
  if (modules_initialized_) {
    deinitDefaultModules();
  }
  if (core_initialized_) {
    T_DjiReturnCode rc = DjiCore_DeInit();
    core_initialized_ = false;
    publishCoreState("deinitialized", rc, "DjiCore_DeInit called");
  }
}

// Load all runtime configuration from the private ROS namespace.  In
// particular, credentials remain external YAML parameters rather than source
// literals, and dangerous commands default to disabled.
bool PsdkRos1Wrapper::loadParams()
{
  private_nh_.param<std::string>("app_name", params_.app_name, "");
  private_nh_.param<std::string>("app_id", params_.app_id, "");
  private_nh_.param<std::string>("app_key", params_.app_key, "");
  private_nh_.param<std::string>("app_license", params_.app_license, "");
  private_nh_.param<std::string>("developer_account", params_.developer_account, "");
  private_nh_.param<std::string>("baudrate", params_.baudrate, "");
  private_nh_.param<std::string>("link_config_file_path", params_.link_config_file_path, "");
  private_nh_.param<std::string>("alias", params_.alias, "PSDK_ROS1");
  private_nh_.param<std::string>("serial_number", params_.serial_number, "PSDKROS1000001");
  private_nh_.param<bool>("auto_start_modules", params_.auto_start_modules, true);
  private_nh_.param<bool>("allow_dangerous_commands", params_.allow_dangerous_commands, false);
  private_nh_.param<int>("core_init_retries", params_.core_init_retries, 1);
  private_nh_.param<std::vector<int> >("firmware_version", params_.firmware_version, std::vector<int>{1, 0, 0, 0});
  private_nh_.param<double>("planner/control_rate", params_.planner_control_rate, 30.0);
  private_nh_.param<double>("planner/command_timeout", params_.planner_command_timeout, 0.3);
  private_nh_.param<std::string>("planner/default_frame", params_.planner_default_frame, "body");
  private_nh_.param<bool>("planner/auto_obtain_authority", params_.planner_auto_obtain_authority, true);
  private_nh_.param<bool>("planner/release_authority_on_disable", params_.planner_release_authority_on_disable, true);
  private_nh_.param<double>("planner/max_horizontal_velocity", params_.planner_max_horizontal_velocity, 5.0);
  private_nh_.param<double>("planner/max_vertical_velocity", params_.planner_max_vertical_velocity, 3.0);
  private_nh_.param<double>("planner/max_yaw_rate", params_.planner_max_yaw_rate, 90.0);
  private_nh_.param<int>("gimbal_feedback/payload_index", params_.gimbal_feedback_payload_index, 1);
  private_nh_.param<int>("gimbal_feedback/frequency", params_.gimbal_feedback_frequency, 50);
  private_nh_.param<double>("mission/default_cruise_speed", params_.mission_default_cruise_speed, 5.0);
  params_.mission_reference_configured =
      private_nh_.getParam("mission/reference_latitude", params_.mission_reference_latitude) &&
      private_nh_.getParam("mission/reference_longitude", params_.mission_reference_longitude) &&
      private_nh_.getParam("mission/reference_altitude", params_.mission_reference_altitude);
  private_nh_.param<std::vector<std::string> >("mission/allow_kmz_file_path_roots",
                                               params_.mission_allow_kmz_file_path_roots,
                                               std::vector<std::string>{"/tmp", "/home"});
  if (params_.planner_control_rate < 1.0) {
    params_.planner_control_rate = 1.0;
  }
  planner_timer_.setPeriod(ros::Duration(1.0 / params_.planner_control_rate), false);
  if (params_.planner_default_frame == "ground") {
    planner_control_mode_.frame = PlannerControlMode::FRAME_GROUND;
  }
  if (params_.gimbal_feedback_payload_index < 1 || params_.gimbal_feedback_payload_index > 7) {
    params_.gimbal_feedback_payload_index = 1;
  }
  if (params_.gimbal_feedback_frequency < 1) {
    params_.gimbal_feedback_frequency = 1;
  }
  g_gimbal_feedback_payload_index = static_cast<uint8_t>(params_.gimbal_feedback_payload_index);
  return true;
}

// Register the OSAL/HAL/socket/filesystem boundary expected by DJI PSDK.  The
// selected link configuration decides whether USB-bulk and network handlers
// are registered in addition to the always-required UART/OSAL handlers.
bool PsdkRos1Wrapper::registerPlatformHandlers()
{
  T_DjiOsalHandler osal_handler = {};
  T_DjiHalUartHandler uart_handler = {};
  T_DjiHalUsbBulkHandler usb_bulk_handler = {};
  T_DjiHalNetworkHandler network_handler = {};
  T_DjiSocketHandler socket_handler = {};
  T_DjiFileSystemHandler file_system_handler = {};

  osal_handler.TaskCreate = Osal_TaskCreate;
  osal_handler.TaskDestroy = Osal_TaskDestroy;
  osal_handler.TaskSleepMs = Osal_TaskSleepMs;
  osal_handler.MutexCreate = Osal_MutexCreate;
  osal_handler.MutexDestroy = Osal_MutexDestroy;
  osal_handler.MutexLock = Osal_MutexLock;
  osal_handler.MutexUnlock = Osal_MutexUnlock;
  osal_handler.SemaphoreCreate = Osal_SemaphoreCreate;
  osal_handler.SemaphoreDestroy = Osal_SemaphoreDestroy;
  osal_handler.SemaphoreWait = Osal_SemaphoreWait;
  osal_handler.SemaphoreTimedWait = Osal_SemaphoreTimedWait;
  osal_handler.SemaphorePost = Osal_SemaphorePost;
  osal_handler.Malloc = Osal_Malloc;
  osal_handler.Free = Osal_Free;
  osal_handler.GetTimeMs = Osal_GetTimeMs;
  osal_handler.GetTimeUs = Osal_GetTimeUs;
  osal_handler.GetRandomNum = Osal_GetRandomNum;

  uart_handler.UartInit = HalUart_Init;
  uart_handler.UartDeInit = HalUart_DeInit;
  uart_handler.UartWriteData = HalUart_WriteData;
  uart_handler.UartReadData = HalUart_ReadData;
  uart_handler.UartGetStatus = HalUart_GetStatus;
  // Manifold2 does not provide the optional USB-UART device-info callback.

  usb_bulk_handler.UsbBulkInit = HalUsbBulk_Init;
  usb_bulk_handler.UsbBulkDeInit = HalUsbBulk_DeInit;
  usb_bulk_handler.UsbBulkWriteData = HalUsbBulk_WriteData;
  usb_bulk_handler.UsbBulkReadData = HalUsbBulk_ReadData;
  usb_bulk_handler.UsbBulkGetDeviceInfo = HalUsbBulk_GetDeviceInfo;

  network_handler.NetworkInit = HalNetWork_Init;
  network_handler.NetworkDeInit = HalNetWork_DeInit;
  network_handler.NetworkGetDeviceInfo = HalNetWork_GetDeviceInfo;

  socket_handler.Socket = Osal_Socket;
  socket_handler.Bind = Osal_Bind;
  socket_handler.Close = Osal_Close;
  socket_handler.UdpSendData = Osal_UdpSendData;
  socket_handler.UdpRecvData = Osal_UdpRecvData;
  socket_handler.TcpListen = Osal_TcpListen;
  socket_handler.TcpAccept = Osal_TcpAccept;
  socket_handler.TcpConnect = Osal_TcpConnect;
  socket_handler.TcpSendData = Osal_TcpSendData;
  socket_handler.TcpRecvData = Osal_TcpRecvData;

  file_system_handler.FileOpen = Osal_FileOpen;
  file_system_handler.FileClose = Osal_FileClose;
  file_system_handler.FileWrite = Osal_FileWrite;
  file_system_handler.FileRead = Osal_FileRead;
  file_system_handler.FileSync = Osal_FileSync;
  file_system_handler.FileSeek = Osal_FileSeek;
  file_system_handler.DirOpen = Osal_DirOpen;
  file_system_handler.DirClose = Osal_DirClose;
  file_system_handler.DirRead = Osal_DirRead;
  file_system_handler.Mkdir = Osal_Mkdir;
  file_system_handler.Unlink = Osal_Unlink;
  file_system_handler.Rename = Osal_Rename;
  file_system_handler.Stat = Osal_Stat;

  if (DjiPlatform_RegOsalHandler(&osal_handler) != kSuccess ||
      DjiPlatform_RegHalUartHandler(&uart_handler) != kSuccess ||
      DjiPlatform_RegSocketHandler(&socket_handler) != kSuccess ||
      DjiPlatform_RegFileSystemHandler(&file_system_handler) != kSuccess) {
    return false;
  }

  if (!params_.link_config_file_path.empty() &&
      DjiUserConfigManager_LoadConfiguration(params_.link_config_file_path.c_str()) == kSuccess) {
    T_DjiUserLinkConfig link_config = {};
    DjiUserConfigManager_GetLinkConfig(&link_config);
    if (link_config.type == DJI_USER_LINK_CONFIG_USE_UART_AND_USB_BULK_DEVICE ||
        link_config.type == DJI_USER_LINK_CONFIG_USE_ONLY_USB_BULK_DEVICE) {
      DjiPlatform_RegHalUsbBulkHandler(&usb_bulk_handler);
    }
    if (link_config.type == DJI_USER_LINK_CONFIG_USE_UART_AND_NETWORK_DEVICE ||
        link_config.type == DJI_USER_LINK_CONFIG_USE_ONLY_NETWORK_DEVICE) {
      DjiPlatform_RegHalNetworkHandler(&network_handler);
    }
  } else {
    DjiPlatform_RegHalUsbBulkHandler(&usb_bulk_handler);
    DjiPlatform_RegHalNetworkHandler(&network_handler);
  }

  platform_ready_ = true;
  publishCoreState("platform_ready", kSuccess, "PSDK platform handlers registered");
  return true;
}

// Copy runtime credentials and link settings into the fixed-size DJI struct
// only after checking all destination capacities; no credential values are
// logged or published by this function.
bool PsdkRos1Wrapper::fillUserInfo(T_DjiUserInfo* user_info) const
{
  std::memset(user_info, 0, sizeof(*user_info));
  if (params_.app_name.size() >= sizeof(user_info->appName) ||
      params_.app_id.size() > sizeof(user_info->appId) ||
      params_.app_key.size() > sizeof(user_info->appKey) ||
      params_.app_license.size() > sizeof(user_info->appLicense) ||
      params_.developer_account.size() >= sizeof(user_info->developerAccount) ||
      params_.baudrate.size() > sizeof(user_info->baudRate)) {
    return false;
  }
  std::strncpy(user_info->appName, params_.app_name.c_str(), sizeof(user_info->appName) - 1);
  std::memcpy(user_info->appId, params_.app_id.c_str(), params_.app_id.size());
  std::memcpy(user_info->appKey, params_.app_key.c_str(), params_.app_key.size());
  std::memcpy(user_info->appLicense, params_.app_license.c_str(), params_.app_license.size());
  std::strncpy(user_info->developerAccount, params_.developer_account.c_str(), sizeof(user_info->developerAccount) - 1);
  std::memcpy(user_info->baudRate, params_.baudrate.c_str(), params_.baudrate.size());
  return true;
}

// Initialize core once, retrying DjiCore_Init according to configuration, then
// apply firmware/alias/serial metadata and start the PSDK application layer.
bool PsdkRos1Wrapper::initCore()
{
  if (core_initialized_) {
    return true;
  }
  if (!platform_ready_ && !registerPlatformHandlers()) {
    return false;
  }

  T_DjiUserInfo user_info;
  if (!fillUserInfo(&user_info)) {
    publishCoreState("user_info_error", DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER, "PSDK user info is too long");
    return false;
  }

  T_DjiReturnCode rc = DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
  for (int attempt = 0; attempt <= params_.core_init_retries; ++attempt) {
    rc = DjiCore_Init(&user_info);
    if (rc == kSuccess) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
  if (rc != kSuccess) {
    publishCoreState("core_init_error", rc, "DjiCore_Init failed");
    return false;
  }

  T_DjiFirmwareVersion firmware = {};
  if (params_.firmware_version.size() >= 4) {
    firmware.majorVersion = static_cast<uint8_t>(params_.firmware_version[0]);
    firmware.minorVersion = static_cast<uint8_t>(params_.firmware_version[1]);
    firmware.modifyVersion = static_cast<uint8_t>(params_.firmware_version[2]);
    firmware.debugVersion = static_cast<uint8_t>(params_.firmware_version[3]);
    DjiCore_SetFirmwareVersion(firmware);
  }

  DjiCore_SetAlias(params_.alias.c_str());
  DjiCore_SetSerialNumber(params_.serial_number.c_str());
  rc = DjiCore_ApplicationStart();
  core_initialized_ = (rc == kSuccess);
  publishCoreState(core_initialized_ ? "started" : "start_error", rc, "DjiCore_ApplicationStart called");

  if (core_initialized_) {
    dispatchAircraftInfo("get_base_info", "{}");
    dispatchAircraftInfo("get_connection_status", "{}");
  }
  return core_initialized_;
}

// Default module startup is deliberately ordered through the common lifecycle
// dispatcher so every module produces the same state/result topic records.
bool PsdkRos1Wrapper::initDefaultModules()
{
  const std::vector<std::string> modules = {
    "fc_subscription", "camera_manager", "gimbal_manager", "liveview", "hms_manager",
    "perception", "low_speed_data_channel", "mop_channel", "waypoint_v2",
    "waypoint_v3", "interest_point", "positioning", "time_sync", "power_management",
    "tethered_battery", "widget_manager", "widget", "xport", "payload_camera",
    "payload_gimbal"
  };
  bool all_ok = true;
  for (const std::string& module : modules) {
    ApiResult result = dispatchModuleLifecycle(module, "init", "{}");
    publishApiResult(module, "init", "{}", result);
    all_ok = all_ok && result.success;
  }
  modules_initialized_ = true;
  return all_ok;
}

// Deinitialize in the reverse dependency order and unsubscribe FC feedback
// before tearing down the subscription module itself.
bool PsdkRos1Wrapper::deinitDefaultModules()
{
  const std::vector<std::string> modules = {
    "payload_gimbal", "tethered_battery", "power_management", "perception", "hms_manager",
    "liveview", "gimbal_manager", "camera_manager", "flight_controller", "fc_subscription",
    "waypoint_v3", "waypoint_v2", "interest_point", "low_speed_data_channel", "mop_channel", "xport"
  };
  for (const std::string& module : modules) {
    if (module == "fc_subscription") {
      unsubscribeGimbalFeedback();
    }
    ApiResult result = dispatchModuleLifecycle(module, "deinit", "{}");
    publishApiResult(module, "deinit", "{}", result);
  }
  modules_initialized_ = false;
  return true;
}

// Subscribe only once per feedback topic.  The callback bridge publishes
// gimbal angles/status directly to ROS while this class tracks subscription state.
bool PsdkRos1Wrapper::subscribeGimbalFeedback()
{
  bool ok = true;
  const E_DjiDataSubscriptionTopicFreq frequency = normalizeSubscriptionFrequency(params_.gimbal_feedback_frequency);
  if (!gimbal_angles_feedback_subscribed_) {
    const E_DjiFcSubscriptionTopic topic =
        gimbalAnglesTopicForPayload(static_cast<uint8_t>(params_.gimbal_feedback_payload_index));
    const T_DjiReturnCode rc = DjiFcSubscription_SubscribeTopic(topic, frequency, gimbalAnglesCallback);
    gimbal_angles_feedback_subscribed_ = (rc == kSuccess);
    ok = ok && gimbal_angles_feedback_subscribed_;
    publishModuleState("fc_subscription", rc == kSuccess, rc == kSuccess, rc, "subscribe_gimbal_angles",
                       "gimbal angle feedback subscription requested");
  }

  if (!gimbal_status_feedback_subscribed_) {
    const T_DjiReturnCode rc = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_STATUS,
                                                               frequency, gimbalStatusCallback);
    gimbal_status_feedback_subscribed_ = (rc == kSuccess);
    ok = ok && gimbal_status_feedback_subscribed_;
    publishModuleState("fc_subscription", rc == kSuccess, rc == kSuccess, rc, "subscribe_gimbal_status",
                       "gimbal status feedback subscription requested");
  }
  return ok;
}

void PsdkRos1Wrapper::unsubscribeGimbalFeedback()
{
  if (gimbal_angles_feedback_subscribed_) {
    const E_DjiFcSubscriptionTopic topic =
        gimbalAnglesTopicForPayload(static_cast<uint8_t>(params_.gimbal_feedback_payload_index));
    DjiFcSubscription_UnSubscribeTopic(topic);
    gimbal_angles_feedback_subscribed_ = false;
  }

  if (gimbal_status_feedback_subscribed_) {
    DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_STATUS);
    gimbal_status_feedback_subscribed_ = false;
  }
}

// Payload slots are numbered by the DJI SDK; map ROS's 1-based parameter to
// the matching subscription topic and fall back to slot 1 for invalid input.
E_DjiFcSubscriptionTopic PsdkRos1Wrapper::gimbalAnglesTopicForPayload(uint8_t payload_index) const
{
  switch (payload_index) {
    case 2:
      return DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES_ON_POS_NO2;
    case 3:
      return DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES_ON_POS_NO3;
    case 4:
      return DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES_ON_POS_NO4;
    case 5:
      return DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES_ON_POS_NO5;
    case 6:
      return DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES_ON_POS_NO6;
    case 7:
      return DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES_ON_POS_NO7;
    case 1:
    default:
      return DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES_ON_POS_NO1;
  }
}

// The typed core services are thin lifecycle controls.  Detailed PSDK return
// codes are mirrored through the core state/result topics by the called stages.
bool PsdkRos1Wrapper::handleInitialize(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
{
  res.success = initCore();
  res.message = res.success ? "PSDK core initialized" : "PSDK core initialization failed";
  return true;
}

bool PsdkRos1Wrapper::handleShutdown(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
{
  shutdown();
  res.success = true;
  res.message = "PSDK shutdown sequence completed";
  return true;
}

// Generic JSON service: dispatch once, copy the result into the response, and
// publish the same request/result pair for asynchronous ROS observers.
bool PsdkRos1Wrapper::handleApi(PsdkApi::Request& req, PsdkApi::Response& res)
{
  ApiResult result = dispatchApi(req.module, req.function, req.request_json);
  res.success = result.success;
  res.return_code = result.return_code;
  res.response_json = result.response_json;
  res.message = result.message;
  publishApiResult(req.module, req.function, req.request_json, result);
  return true;
}

// The following typed camera/gimbal/flight-controller services reuse the same
// dispatcher as /api/call, keeping behavior and result publication consistent.
bool PsdkRos1Wrapper::handleCameraFormatSdCard(CameraFormatSdCard::Request& req, CameraFormatSdCard::Response& res)
{
  std::ostringstream json;
  json << "{\"payload_index\":" << static_cast<int>(req.payload_index) << "}";
  ApiResult result = dispatchCameraManager("format_storage", json.str());
  publishApiResult("camera_manager", "format_storage", json.str(), result);
  res.success = result.success;
  return true;
}

bool PsdkRos1Wrapper::handleCameraGetType(CameraGetType::Request& req, CameraGetType::Response& res)
{
  std::ostringstream json;
  json << "{\"payload_index\":" << static_cast<int>(req.payload_index) << "}";
  ApiResult result = dispatchCameraManager("get_camera_type", json.str());
  publishApiResult("camera_manager", "get_camera_type", json.str(), result);
  res.success = result.success;
  res.camera_type = result.response_json;
  return true;
}

bool PsdkRos1Wrapper::handleCameraRecordVideo(CameraRecordVideo::Request& req, CameraRecordVideo::Response& res)
{
  std::ostringstream json;
  json << "{\"payload_index\":" << static_cast<int>(req.payload_index) << "}";
  ApiResult result = dispatchCameraManager(req.start_stop ? "start_record_video" : "stop_record_video", json.str());
  publishApiResult("camera_manager", req.start_stop ? "start_record_video" : "stop_record_video", json.str(), result);
  res.success = result.success;
  return true;
}

bool PsdkRos1Wrapper::handleCameraSetupStreaming(CameraSetupStreaming::Request& req, CameraSetupStreaming::Response& res)
{
  std::ostringstream json;
  json << "{\"camera_position\":" << static_cast<int>(req.payload_index)
       << ",\"camera_source\":" << static_cast<int>(req.camera_source)
       << ",\"publish_raw\":false}";
  ApiResult result = dispatchLiveview(req.start_stop ? "start_h264_stream" : "stop_h264_stream", json.str());
  publishApiResult("liveview", req.start_stop ? "start_h264_stream" : "stop_h264_stream", json.str(), result);
  res.success = result.success;
  return true;
}

bool PsdkRos1Wrapper::handleCameraShootSinglePhoto(CameraShootSinglePhoto::Request& req, CameraShootSinglePhoto::Response& res)
{
  std::ostringstream json;
  json << "{\"payload_index\":" << static_cast<int>(req.payload_index) << ",\"mode\":1}";
  ApiResult result = dispatchCameraManager("start_shoot_photo", json.str());
  publishApiResult("camera_manager", "start_shoot_photo", json.str(), result);
  res.success = result.success;
  return true;
}

bool PsdkRos1Wrapper::handleGimbalSetMode(GimbalSetMode::Request& req, GimbalSetMode::Response& res)
{
  std::ostringstream json;
  json << "{\"payload_index\":" << static_cast<int>(req.payload_index)
       << ",\"mode\":" << static_cast<int>(req.gimbal_mode) << "}";
  ApiResult result = dispatchGimbalManager("set_mode", json.str());
  publishApiResult("gimbal_manager", "set_mode", json.str(), result);
  res.success = result.success;
  return true;
}

bool PsdkRos1Wrapper::handleGimbalReset(GimbalReset::Request& req, GimbalReset::Response& res)
{
  std::ostringstream json;
  json << "{\"payload_index\":" << static_cast<int>(req.payload_index)
       << ",\"reset_mode\":" << static_cast<int>(req.reset_mode) << "}";
  ApiResult result = dispatchGimbalManager("reset", json.str());
  publishApiResult("gimbal_manager", "reset", json.str(), result);
  res.success = result.success;
  return true;
}

bool PsdkRos1Wrapper::handleSetGoHomeAltitude(SetGoHomeAltitude::Request& req, SetGoHomeAltitude::Response& res)
{
  std::ostringstream json;
  json << "{\"altitude\":" << req.altitude << "}";
  ApiResult result = dispatchFlightController("set_go_home_altitude", json.str());
  publishApiResult("flight_controller", "set_go_home_altitude", json.str(), result);
  res.success = result.success;
  return true;
}

bool PsdkRos1Wrapper::handleGetGoHomeAltitude(GetGoHomeAltitude::Request&, GetGoHomeAltitude::Response& res)
{
  ApiResult result = dispatchFlightController("get_go_home_altitude", "{}");
  publishApiResult("flight_controller", "get_go_home_altitude", "{}", result);
  res.success = result.success;
  res.altitude = static_cast<uint16_t>(jsonInt(result.response_json, "altitude", 0));
  return true;
}

// Gimbal topic callbacks translate SI/radian ROS commands to the DJI JSON
// convention (degrees for angles) and publish a typed command-result message.
void PsdkRos1Wrapper::gimbalRotationCb(const GimbalRotation::ConstPtr& msg)
{
  std::ostringstream json;
  json << "{\"payload_index\":" << static_cast<int>(msg->payload_index)
       << ",\"rotation_mode\":" << static_cast<int>(msg->rotation_mode)
       << ",\"pitch_deg\":" << msg->pitch * kRadToDeg
       << ",\"roll_deg\":" << msg->roll * kRadToDeg
       << ",\"yaw_deg\":" << msg->yaw * kRadToDeg
       << ",\"time\":" << msg->time << "}";
  ApiResult result = dispatchGimbalManager("rotate", json.str());
  publishApiResult("gimbal_manager", "rotate", json.str(), result);
  PsdkApiResult out;
  out.header.stamp = ros::Time::now();
  out.module = "gimbal_manager";
  out.function = "rotate";
  out.request_json = json.str();
  out.success = result.success;
  out.return_code = result.return_code;
  out.response_json = result.response_json;
  out.message = result.message;
  gimbal_command_result_pub_.publish(out);
}

void PsdkRos1Wrapper::gimbalModeCb(const GimbalMode::ConstPtr& msg)
{
  std::ostringstream json;
  json << "{\"payload_index\":" << static_cast<int>(msg->payload_index)
       << ",\"mode\":" << static_cast<int>(msg->gimbal_mode) << "}";
  ApiResult result = dispatchGimbalManager("set_mode", json.str());
  publishApiResult("gimbal_manager", "set_mode", json.str(), result);
  PsdkApiResult out;
  out.header.stamp = ros::Time::now();
  out.module = "gimbal_manager";
  out.function = "set_mode";
  out.request_json = json.str();
  out.success = result.success;
  out.return_code = result.return_code;
  out.response_json = result.response_json;
  out.message = result.message;
  gimbal_command_result_pub_.publish(out);
}

void PsdkRos1Wrapper::gimbalResetCb(const GimbalResetCmd::ConstPtr& msg)
{
  std::ostringstream json;
  json << "{\"payload_index\":" << static_cast<int>(msg->payload_index)
       << ",\"reset_mode\":" << static_cast<int>(msg->reset_mode) << "}";
  ApiResult result = dispatchGimbalManager("reset", json.str());
  publishApiResult("gimbal_manager", "reset", json.str(), result);
  PsdkApiResult out;
  out.header.stamp = ros::Time::now();
  out.module = "gimbal_manager";
  out.function = "reset";
  out.request_json = json.str();
  out.success = result.success;
  out.return_code = result.return_code;
  out.response_json = result.response_json;
  out.message = result.message;
  gimbal_command_result_pub_.publish(out);
}

// Planner velocity and pose inputs are mutually exclusive latest-command
// caches.  plannerTimerCb later applies the timeout/zero-hold policy.
void PsdkRos1Wrapper::plannerVelocityCb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  latest_velocity_cmd_ = *msg;
  latest_planner_cmd_time_ = ros::Time::now();
  planner_has_velocity_cmd_ = true;
  planner_has_pose_cmd_ = false;
}

void PsdkRos1Wrapper::plannerPoseCb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  latest_pose_cmd_ = *msg;
  latest_planner_cmd_time_ = ros::Time::now();
  planner_has_pose_cmd_ = true;
  planner_has_velocity_cmd_ = false;
}

// Enabling obtains joystick authority (when configured) and starts the timer;
// disabling sends one zero command before optionally releasing that authority.
void PsdkRos1Wrapper::plannerEnableCb(const std_msgs::Bool::ConstPtr& msg)
{
  if (msg->data && !planner_enabled_) {
    configureJoystickMode();
    T_DjiReturnCode rc = kSuccess;
    if (params_.planner_auto_obtain_authority) {
      rc = DjiFlightController_ObtainJoystickCtrlAuthority();
      planner_has_authority_ = (rc == kSuccess);
    }
    planner_emergency_stop_ = false;
    latest_planner_cmd_time_ = ros::Time::now();
    const bool authority_ok = !params_.planner_auto_obtain_authority || planner_has_authority_;
    planner_enabled_ = authority_ok;
    if (authority_ok) {
      planner_timer_.start();
    }
    publishPlannerState(authority_ok ? PlannerState::ENABLED : PlannerState::ERROR,
                        planner_has_authority_, false, "planner enabled", rc);
    PsdkApiResult result_msg;
    result_msg.header.stamp = ros::Time::now();
    result_msg.module = "flight_controller";
    result_msg.function = "obtain_joystick_authority";
    result_msg.request_json = "{}";
    result_msg.success = (rc == kSuccess);
    result_msg.return_code = rc;
    result_msg.response_json = resultJson(rc);
    result_msg.message = "planner enabled";
    planner_command_result_pub_.publish(result_msg);
  } else if (!msg->data && planner_enabled_) {
    T_DjiFlightControllerJoystickCommand zero = buildJoystickCommand(true);
    T_DjiReturnCode rc = DjiFlightController_ExecuteJoystickAction(zero);
    if (params_.planner_release_authority_on_disable && planner_has_authority_) {
      rc = DjiFlightController_ReleaseJoystickCtrlAuthority();
      planner_has_authority_ = false;
    }
    planner_enabled_ = false;
    planner_timer_.stop();
    publishPlannerState(PlannerState::DISABLED, planner_has_authority_, false, "planner disabled", rc);
    PsdkApiResult result_msg;
    result_msg.header.stamp = ros::Time::now();
    result_msg.module = "flight_controller";
    result_msg.function = "release_joystick_authority";
    result_msg.request_json = "{}";
    result_msg.success = (rc == kSuccess);
    result_msg.return_code = rc;
    result_msg.response_json = resultJson(rc);
    result_msg.message = "planner disabled";
    planner_command_result_pub_.publish(result_msg);
  }
}

// Emergency stop is latched locally, stops planner output immediately, and
// still publishes the DJI return code so operators can diagnose rejection.
void PsdkRos1Wrapper::plannerEmergencyStopCb(const std_msgs::Bool::ConstPtr& msg)
{
  if (!msg->data) {
    return;
  }
  planner_emergency_stop_ = true;
  planner_enabled_ = false;
  planner_timer_.stop();
  T_DjiReturnCode rc = DjiFlightController_ExecuteEmergencyBrakeAction();
  publishPlannerState(PlannerState::EMERGENCY_STOP, planner_has_authority_, false, "emergency brake requested", rc);
  PsdkApiResult result_msg;
  result_msg.header.stamp = ros::Time::now();
  result_msg.module = "flight_controller";
  result_msg.function = "emergency_brake";
  result_msg.request_json = "{}";
  result_msg.success = (rc == kSuccess);
  result_msg.return_code = rc;
  result_msg.response_json = resultJson(rc);
  result_msg.message = "emergency brake requested";
  planner_command_result_pub_.publish(result_msg);
}

// Changes to control mode take effect immediately when the planner is active;
// otherwise the selected mode is retained for the next enable transition.
void PsdkRos1Wrapper::plannerControlModeCb(const PlannerControlMode::ConstPtr& msg)
{
  planner_control_mode_ = *msg;
  if (planner_enabled_) {
    configureJoystickMode();
  }
}

// The planner timer emits either the newest fresh command or a zero hold after
// command_timeout.  This is a safety boundary, not just a periodic publisher.
void PsdkRos1Wrapper::plannerTimerCb(const ros::TimerEvent&)
{
  if (!planner_enabled_ || planner_emergency_stop_) {
    return;
  }

  const bool fresh = (ros::Time::now() - latest_planner_cmd_time_).toSec() <= params_.planner_command_timeout;
  T_DjiFlightControllerJoystickCommand cmd = buildJoystickCommand(!fresh);
  T_DjiReturnCode rc = DjiFlightController_ExecuteJoystickAction(cmd);
  PsdkApiResult result_msg;
  result_msg.header.stamp = ros::Time::now();
  result_msg.module = "flight_controller";
  result_msg.function = "joystick";
  result_msg.request_json = fresh ? "{\"source\":\"planner\"}" : "{\"source\":\"planner\",\"timeout\":true}";
  result_msg.success = (rc == kSuccess);
  result_msg.return_code = rc;
  result_msg.response_json = resultJson(rc);
  result_msg.message = fresh ? "planner joystick command" : "planner timeout zero hold";
  planner_command_result_pub_.publish(result_msg);

  JoystickCommand echo;
  echo.header.stamp = ros::Time::now();
  echo.x = cmd.x;
  echo.y = cmd.y;
  echo.z = cmd.z;
  echo.yaw = cmd.yaw;
  echo.horizontal_mode = planner_control_mode_.horizontal_mode;
  echo.vertical_mode = planner_control_mode_.vertical_mode;
  echo.yaw_mode = planner_control_mode_.yaw_mode;
  echo.frame = planner_control_mode_.frame;
  echo.stable_mode = planner_control_mode_.stable_mode;
  joystick_cmd_echo_pub_.publish(echo);

  publishPlannerState(fresh ? PlannerState::ACTIVE : PlannerState::TIMEOUT,
                      planner_has_authority_, fresh,
                      fresh ? "planner command active" : "planner command timeout, sending zero hold",
                      rc);
}

// Mission inputs are cached first; MissionCommand decides when cached path/KMZ
// data is validated, converted, uploaded, started, or cleared.
void PsdkRos1Wrapper::missionPathCb(const nav_msgs::Path::ConstPtr& msg)
{
  latest_mission_path_ = *msg;
  mission_has_path_ = true;
  publishMissionState(MissionState::PATH_READY, msg->header.frame_id,
                      static_cast<uint32_t>(msg->poses.size()),
                      params_.mission_default_cruise_speed,
                      "path cached; send MissionCommand UPLOAD_PATH to upload", kSuccess);
}

void PsdkRos1Wrapper::missionKmzFileCb(const KmzFile::ConstPtr& msg)
{
  latest_kmz_file_name_ = msg->file_name;
  latest_kmz_data_ = msg->data;
  mission_has_kmz_ = !latest_kmz_data_.empty();
  publishMissionState(mission_has_kmz_ ? MissionState::KMZ_READY : MissionState::ERROR,
                      latest_kmz_file_name_, 0, params_.mission_default_cruise_speed,
                      mission_has_kmz_ ? "KMZ data cached" : "empty KMZ data", kSuccess);
}

void PsdkRos1Wrapper::missionKmzPathCb(const std_msgs::String::ConstPtr& msg)
{
  latest_kmz_path_ = msg->data;
  mission_has_kmz_ = false;
  publishMissionState(MissionState::KMZ_READY, latest_kmz_path_, 0,
                      params_.mission_default_cruise_speed,
                      "KMZ path cached; send MissionCommand UPLOAD_KMZ to read and upload", kSuccess);
}

void PsdkRos1Wrapper::missionCommandCb(const MissionCommand::ConstPtr& msg)
{
  T_DjiReturnCode rc = kSuccess;
  std::string message = "mission command handled";
  uint8_t state = MissionState::IDLE;
  const float cruise_speed = msg->cruise_speed > 0.0f ? msg->cruise_speed : params_.mission_default_cruise_speed;

  if (msg->command == MissionCommand::UPLOAD_PATH) {
    if (!mission_has_path_ || latest_mission_path_.poses.empty()) {
      rc = DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
      state = MissionState::ERROR;
      message = "no non-empty nav_msgs/Path cached";
    } else if (latest_mission_path_.poses.size() > 65535) {
      rc = DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
      state = MissionState::ERROR;
      message = "Path has more than 65535 waypoints";
    } else if (!params_.mission_reference_configured) {
      rc = DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
      state = MissionState::ERROR;
      message = "mission reference latitude/longitude/altitude is not configured";
    } else {
      std::vector<T_DjiWaypointV2> waypoints(latest_mission_path_.poses.size());
      const double earth_radius_m = 6378137.0;
      const double ref_lat_rad = params_.mission_reference_latitude / kRadToDeg;
      for (size_t i = 0; i < latest_mission_path_.poses.size(); ++i) {
        const geometry_msgs::Point& p = latest_mission_path_.poses[i].pose.position;
        const double north_m = p.x;
        const double east_m = p.y;
        waypoints[i] = T_DjiWaypointV2();
        waypoints[i].latitude = params_.mission_reference_latitude + (north_m / earth_radius_m) * kRadToDeg;
        waypoints[i].longitude = params_.mission_reference_longitude +
            (east_m / (earth_radius_m * std::cos(ref_lat_rad))) * kRadToDeg;
        waypoints[i].relativeHeight = static_cast<dji_f32_t>(p.z);
        waypoints[i].waypointType = DJI_WAYPOINT_V2_FLIGHT_PATH_MODE_GO_TO_POINT_IN_STRAIGHT_AND_STOP;
        waypoints[i].headingMode = DJI_WAYPOINT_V2_HEADING_MODE_AUTO;
        waypoints[i].config.useLocalCruiseVel = 1;
        waypoints[i].config.useLocalMaxVel = 1;
        waypoints[i].dampingDistance = 0;
        waypoints[i].heading = 0;
        waypoints[i].turnMode = DJI_WAYPOINT_V2_TURN_MODE_CLOCK_WISE;
        waypoints[i].pointOfInterest = T_DjiWaypointV2RelativePosition();
        waypoints[i].maxFlightSpeed = clampValue(cruise_speed, 2.0, 15.0);
        waypoints[i].autoFlightSpeed = clampValue(cruise_speed, -15.0, 15.0);
      }

      T_DjiWayPointV2MissionSettings mission = {};
      mission.missionID = 1;
      mission.repeatTimes = 0;
      mission.finishedAction = DJI_WAYPOINT_V2_FINISHED_NO_ACTION;
      mission.maxFlightSpeed = clampValue(cruise_speed, 2.0, 15.0);
      mission.autoFlightSpeed = clampValue(cruise_speed, -15.0, 15.0);
      mission.actionWhenRcLost = DJI_WAYPOINT_V2_MISSION_STOP_WAYPOINT_V2_AND_EXECUTE_RC_LOST_ACTION;
      mission.gotoFirstWaypointMode = DJI_WAYPOINT_V2_MISSION_GO_TO_FIRST_WAYPOINT_MODE_SAFELY;
      mission.mission = waypoints.data();
      mission.missTotalLen = static_cast<uint16_t>(waypoints.size());
      mission.actionList.actions = nullptr;
      mission.actionList.actionNum = 0;

      rc = DjiWaypointV2_UploadMission(&mission);
      if (rc == kSuccess) {
        mission_has_kmz_ = false;
      }
      state = rc == kSuccess ? MissionState::READY : MissionState::ERROR;
      message = "Path converted to WaypointV2 mission and upload requested";
    }
  } else if (msg->command == MissionCommand::UPLOAD_KMZ) {
    std::vector<uint8_t> data = latest_kmz_data_;
    if (data.empty() && !latest_kmz_path_.empty()) {
      if (!isKmzPathAllowed(latest_kmz_path_)) {
        rc = DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
        state = MissionState::ERROR;
        message = "KMZ path is outside allowed roots";
      } else {
        FILE* file = std::fopen(latest_kmz_path_.c_str(), "rb");
        if (file == nullptr) {
          rc = DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
          state = MissionState::ERROR;
          message = "failed to open KMZ file";
        } else {
          std::fseek(file, 0, SEEK_END);
          long size = std::ftell(file);
          std::fseek(file, 0, SEEK_SET);
          if (size > 0) {
            data.resize(static_cast<size_t>(size));
            std::fread(data.data(), 1, data.size(), file);
          }
          std::fclose(file);
        }
      }
    }
    if (rc == kSuccess) {
      if (data.empty()) {
        rc = DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
        state = MissionState::ERROR;
        message = "no KMZ data cached";
      } else {
        rc = DjiWaypointV3_UploadKmzFile(data.data(), static_cast<uint32_t>(data.size()));
        mission_has_kmz_ = (rc == kSuccess);
        state = rc == kSuccess ? MissionState::READY : MissionState::ERROR;
        message = "KMZ upload requested";
      }
    }
  } else if (msg->command == MissionCommand::START) {
    rc = mission_has_kmz_ ? DjiWaypointV3_Action(DJI_WAYPOINT_V3_ACTION_START) : DjiWaypointV2_Start();
    state = rc == kSuccess ? MissionState::RUNNING : MissionState::ERROR;
  } else if (msg->command == MissionCommand::PAUSE) {
    rc = mission_has_kmz_ ? DjiWaypointV3_Action(DJI_WAYPOINT_V3_ACTION_PAUSE) : DjiWaypointV2_Pause();
    state = rc == kSuccess ? MissionState::PAUSED : MissionState::ERROR;
  } else if (msg->command == MissionCommand::RESUME) {
    rc = mission_has_kmz_ ? DjiWaypointV3_Action(DJI_WAYPOINT_V3_ACTION_RESUME) : DjiWaypointV2_Resume();
    state = rc == kSuccess ? MissionState::RUNNING : MissionState::ERROR;
  } else if (msg->command == MissionCommand::STOP) {
    rc = mission_has_kmz_ ? DjiWaypointV3_Action(DJI_WAYPOINT_V3_ACTION_STOP) : DjiWaypointV2_Stop();
    state = rc == kSuccess ? MissionState::STOPPED : MissionState::ERROR;
  } else if (msg->command == MissionCommand::SET_CRUISE_SPEED) {
    rc = DjiWaypointV2_SetGlobalCruiseSpeed(cruise_speed);
    state = rc == kSuccess ? MissionState::READY : MissionState::ERROR;
  } else if (msg->command == MissionCommand::CLEAR) {
    latest_mission_path_ = nav_msgs::Path();
    latest_kmz_data_.clear();
    latest_kmz_file_name_.clear();
    latest_kmz_path_.clear();
    mission_has_path_ = false;
    mission_has_kmz_ = false;
    state = MissionState::IDLE;
    message = "mission cache cleared";
  } else {
    rc = DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    state = MissionState::ERROR;
    message = "unknown mission command";
  }

  publishMissionState(state, msg->mission_id,
                      mission_has_path_ ? static_cast<uint32_t>(latest_mission_path_.poses.size()) : 0,
                      cruise_speed, message, rc);
  publishEvent("mission", "command", rc, numberJson("command", static_cast<int>(msg->command)), message);
  PsdkEvent mission_event;
  mission_event.header.stamp = ros::Time::now();
  mission_event.module = "mission";
  mission_event.event_name = "command";
  mission_event.return_code = rc;
  mission_event.payload_json = numberJson("command", static_cast<int>(msg->command));
  mission_event.message = message;
  mission_event_pub_.publish(mission_event);
}

// Route a module/function pair to the narrowest dispatcher.  The dispatcher
// intentionally keeps lifecycle calls generic while preserving module-specific
// request parsing and result formatting in the child functions.
PsdkRos1Wrapper::ApiResult PsdkRos1Wrapper::dispatchApi(const std::string& module, const std::string& function,
                                                        const std::string& request_json)
{
  if (module == "core") return dispatchCore(function, request_json);
  if (module == "aircraft_info") return dispatchAircraftInfo(function, request_json);
  if (function == "init" || function == "deinit") return dispatchModuleLifecycle(module, function, request_json);
  if (module == "camera_manager") return dispatchCameraManager(function, request_json);
  if (module == "flight_controller") return dispatchFlightController(function, request_json);
  if (module == "gimbal_manager") return dispatchGimbalManager(function, request_json);
  if (module == "liveview") return dispatchLiveview(function, request_json);
  if (module == "perception") return dispatchPerception(function, request_json);
  if (module.find("data_channel") != std::string::npos || module == "mop_channel" || module == "cloud_api") {
    return dispatchDataChannel(module, function, request_json);
  }
  if (module == "waypoint_v2" || module == "waypoint_v3" || module == "interest_point") {
    return dispatchWaypoint(module, function, request_json);
  }
  return dispatchPowerAndUtility(module, function, request_json);
}

// Core owns the application lifecycle; deinit is kept here so callers using
// the generic API follow the same state publication path as typed services.
PsdkRos1Wrapper::ApiResult PsdkRos1Wrapper::dispatchCore(const std::string& function, const std::string&)
{
  if (function == "init") return {initCore(), kSuccess, "{\"state\":\"started\"}", "core init requested"};
  if (function == "deinit") {
    T_DjiReturnCode rc = DjiCore_DeInit();
    core_initialized_ = false;
    publishCoreState("deinitialized", rc, "DjiCore_DeInit called");
    return {rc == kSuccess, rc, resultJson(rc), "core deinit requested"};
  }
  if (function == "application_start") {
    T_DjiReturnCode rc = DjiCore_ApplicationStart();
    return {rc == kSuccess, rc, resultJson(rc), "application start requested"};
  }
  return {false, kUnsupported, "{}", "unsupported core function"};
}

// Aircraft information is read-only telemetry and is published as compact
// JSON because the package does not define a separate message for every SDK
// version field.
PsdkRos1Wrapper::ApiResult PsdkRos1Wrapper::dispatchAircraftInfo(const std::string& function, const std::string&)
{
  if (function == "get_base_info") {
    T_DjiAircraftInfoBaseInfo info = {};
    T_DjiReturnCode rc = DjiAircraftInfo_GetBaseInfo(&info);
    std::ostringstream json;
    json << "{\"aircraft_series\":" << static_cast<int>(info.aircraftSeries)
         << ",\"mount_position_type\":" << static_cast<int>(info.mountPositionType)
         << ",\"aircraft_type\":" << static_cast<int>(info.aircraftType)
         << ",\"adapter_type\":" << static_cast<int>(info.djiAdapterType)
         << ",\"mount_position\":" << static_cast<int>(info.mountPosition) << "}";
    publishJson("aircraft", "base_info", json.str());
    return {rc == kSuccess, rc, json.str(), "aircraft base info"};
  }
  if (function == "get_mobile_app_info") {
    T_DjiMobileAppInfo info = {};
    T_DjiReturnCode rc = DjiAircraftInfo_GetMobileAppInfo(&info);
    std::ostringstream json;
    json << "{\"language\":" << static_cast<int>(info.appLanguage)
         << ",\"screen_type\":" << static_cast<int>(info.appScreenType) << "}";
    publishJson("aircraft", "mobile_app_info", json.str());
    return {rc == kSuccess, rc, json.str(), "mobile app info"};
  }
  if (function == "get_connection_status") {
    bool connected = false;
    T_DjiReturnCode rc = DjiAircraftInfo_GetConnectionStatus(&connected);
    std::string json = std::string("{\"connected\":") + boolJson(connected) + "}";
    publishJson("aircraft", "connection_status", json);
    return {rc == kSuccess, rc, json, "connection status"};
  }
  if (function == "get_aircraft_version") {
    T_DjiAircraftVersion version = {};
    T_DjiReturnCode rc = DjiAircraftInfo_GetAircraftVersion(&version);
    std::ostringstream json;
    json << "{\"major\":" << static_cast<int>(version.majorVersion)
         << ",\"minor\":" << static_cast<int>(version.minorVersion)
         << ",\"modify\":" << static_cast<int>(version.modifyVersion)
         << ",\"debug\":" << static_cast<int>(version.debugVersion) << "}";
    publishJson("aircraft", "version", json.str());
    return {rc == kSuccess, rc, json.str(), "aircraft version"};
  }
  return {false, kUnsupported, "{}", "unsupported aircraft_info function"};
}

// Common init/deinit handling centralizes module lifecycle calls and hooks FC
// feedback subscription into the fc_subscription init/deinit transitions.
PsdkRos1Wrapper::ApiResult PsdkRos1Wrapper::dispatchModuleLifecycle(const std::string& module, const std::string& function,
                                                                    const std::string& request_json)
{
  T_DjiReturnCode rc = kUnsupported;
  if (function == "init") {
    if (module == "fc_subscription") rc = DjiFcSubscription_Init();
    else if (module == "camera_manager") rc = DjiCameraManager_Init();
    else if (module == "gimbal_manager") rc = DjiGimbalManager_Init();
    else if (module == "payload_gimbal") rc = DjiGimbal_Init();
    else if (module == "liveview") rc = DjiLiveview_Init();
    else if (module == "hms_manager") rc = DjiHmsManager_Init();
    else if (module == "hms_customization") rc = DjiHmsCustomization_Init();
    else if (module == "perception") rc = DjiPerception_Init();
    else if (module == "low_speed_data_channel") rc = DjiLowSpeedDataChannel_Init();
    else if (module == "mop_channel") rc = DjiMopChannel_Init();
    else if (module == "waypoint_v2") rc = DjiWaypointV2_Init();
    else if (module == "waypoint_v3") rc = DjiWaypointV3_Init();
    else if (module == "interest_point") rc = DjiInterestPoint_Init();
    else if (module == "positioning") rc = DjiPositioning_Init();
    else if (module == "time_sync") rc = DjiTimeSync_Init();
    else if (module == "power_management") rc = DjiPowerManagement_Init();
    else if (module == "tethered_battery") rc = DjiTetheredBattery_Init();
    else if (module == "widget_manager") rc = DjiWidgetManager_Init();
    else if (module == "widget") rc = DjiWidget_Init();
    else if (module == "xport") rc = DjiXPort_Init();
    else if (module == "payload_camera") rc = DjiPayloadCamera_Init();
    else if (module == "flight_controller") {
      T_DjiFlightControllerRidInfo rid = {};
      rid.latitude = jsonDouble(request_json, "latitude", 0.0);
      rid.longitude = jsonDouble(request_json, "longitude", 0.0);
      rid.altitude = static_cast<uint16_t>(jsonInt(request_json, "altitude", 0));
      rc = DjiFlightController_Init(rid);
    }
    publishModuleState(module, rc == kSuccess, rc == kSuccess, rc, "init", "module init requested");
    if (module == "fc_subscription" && rc == kSuccess) {
      subscribeGimbalFeedback();
    }
    return {rc == kSuccess, rc, resultJson(rc), "module init requested"};
  }

  if (function == "deinit") {
    if (module == "fc_subscription") {
      unsubscribeGimbalFeedback();
    }
    if (module == "fc_subscription") rc = DjiFcSubscription_DeInit();
    else if (module == "camera_manager") rc = DjiCameraManager_DeInit();
    else if (module == "gimbal_manager") rc = DjiGimbalManager_Deinit();
    else if (module == "payload_gimbal") rc = DjiGimbal_DeInit();
    else if (module == "liveview") rc = DjiLiveview_Deinit();
    else if (module == "hms_manager") rc = DjiHmsManager_DeInit();
    else if (module == "hms_customization") rc = DjiHmsCustomization_DeInit();
    else if (module == "perception") rc = DjiPerception_Deinit();
    else if (module == "low_speed_data_channel") rc = DjiLowSpeedDataChannel_DeInit();
    else if (module == "waypoint_v2") rc = DjiWaypointV2_Deinit();
    else if (module == "waypoint_v3") rc = DjiWaypointV3_DeInit();
    else if (module == "interest_point") rc = DjiInterestPoint_DeInit();
    else if (module == "power_management") rc = DjiPowerManagement_DeInit();
    else if (module == "tethered_battery") rc = DjiTetheredBattery_DeInit();
    else if (module == "widget_manager") rc = DjiWidgetManager_DeInit();
    else if (module == "xport") rc = DjiXPort_DeInit();
    else if (module == "flight_controller") rc = DjiFlightController_DeInit();
    publishModuleState(module, false, rc == kSuccess, rc, "deinit", "module deinit requested");
    return {rc == kSuccess, rc, resultJson(rc), "module deinit requested"};
  }

  return {false, kUnsupported, "{}", "unsupported lifecycle function"};
}

// Camera manager operations use payload_index to select the mounted camera;
// storage formatting and file deletion remain safety-gated below.
PsdkRos1Wrapper::ApiResult PsdkRos1Wrapper::dispatchCameraManager(const std::string& function, const std::string& request_json)
{
  E_DjiMountPosition pos = static_cast<E_DjiMountPosition>(jsonInt(request_json, "payload_index", 1));
  T_DjiReturnCode rc = kUnsupported;
  std::string json = "{}";

  if (function == "get_camera_type") {
    E_DjiCameraType type;
    rc = DjiCameraManager_GetCameraType(pos, &type);
    json = numberJson("camera_type", static_cast<int>(type));
  } else if (function == "set_mode") {
    rc = DjiCameraManager_SetMode(pos, static_cast<E_DjiCameraManagerWorkMode>(jsonInt(request_json, "mode", 0)));
  } else if (function == "get_mode") {
    E_DjiCameraManagerWorkMode mode;
    rc = DjiCameraManager_GetMode(pos, &mode);
    json = numberJson("mode", static_cast<int>(mode));
  } else if (function == "start_shoot_photo") {
    rc = DjiCameraManager_StartShootPhoto(pos, static_cast<E_DjiCameraManagerShootPhotoMode>(jsonInt(request_json, "mode", 1)));
  } else if (function == "stop_shoot_photo") {
    rc = DjiCameraManager_StopShootPhoto(pos);
  } else if (function == "set_iso") {
    rc = DjiCameraManager_SetISO(pos, static_cast<E_DjiCameraManagerISO>(jsonInt(request_json, "iso", 0)));
  } else if (function == "get_iso") {
    E_DjiCameraManagerISO iso;
    rc = DjiCameraManager_GetISO(pos, &iso);
    json = numberJson("iso", static_cast<int>(iso));
  } else if (function == "set_aperture") {
    rc = DjiCameraManager_SetAperture(pos, static_cast<E_DjiCameraManagerAperture>(jsonInt(request_json, "aperture", 0)));
  } else if (function == "get_aperture") {
    E_DjiCameraManagerAperture aperture;
    rc = DjiCameraManager_GetAperture(pos, &aperture);
    json = numberJson("aperture", static_cast<int>(aperture));
  } else if (function == "set_shutter_speed") {
    rc = DjiCameraManager_SetShutterSpeed(pos, static_cast<E_DjiCameraManagerShutterSpeed>(jsonInt(request_json, "shutter_speed", 0)));
  } else if (function == "get_shutter_speed") {
    E_DjiCameraManagerShutterSpeed speed;
    rc = DjiCameraManager_GetShutterSpeed(pos, &speed);
    json = numberJson("shutter_speed", static_cast<int>(speed));
  } else if (function == "set_exposure_mode") {
    rc = DjiCameraManager_SetExposureMode(pos, static_cast<E_DjiCameraManagerExposureMode>(jsonInt(request_json, "exposure_mode", 1)));
  } else if (function == "get_exposure_mode") {
    E_DjiCameraManagerExposureMode mode;
    rc = DjiCameraManager_GetExposureMode(pos, &mode);
    json = numberJson("exposure_mode", static_cast<int>(mode));
  } else if (function == "set_focus_mode") {
    rc = DjiCameraManager_SetFocusMode(pos, static_cast<E_DjiCameraManagerFocusMode>(jsonInt(request_json, "focus_mode", 1)));
  } else if (function == "get_focus_mode") {
    E_DjiCameraManagerFocusMode mode;
    rc = DjiCameraManager_GetFocusMode(pos, &mode);
    json = numberJson("focus_mode", static_cast<int>(mode));
  } else if (function == "set_focus_target") {
    T_DjiCameraManagerFocusPosData target = {};
    target.focusX = jsonDouble(request_json, "x", 0.5);
    target.focusY = jsonDouble(request_json, "y", 0.5);
    rc = DjiCameraManager_SetFocusTarget(pos, target);
  } else if (function == "get_focus_target") {
    T_DjiCameraManagerFocusPosData target = {};
    rc = DjiCameraManager_GetFocusTarget(pos, &target);
    std::ostringstream ss;
    ss << "{\"x\":" << target.focusX << ",\"y\":" << target.focusY << "}";
    json = ss.str();
  } else if (function == "set_focus_ring_value") {
    rc = DjiCameraManager_SetFocusRingValue(pos, static_cast<uint32_t>(jsonInt(request_json, "value", 0)));
  } else if (function == "get_focus_ring_value") {
    uint16_t value = 0;
    rc = DjiCameraManager_GetFocusRingValue(pos, &value);
    json = numberJson("value", value);
  } else if (function == "set_optical_zoom") {
    rc = DjiCameraManager_SetOpticalZoomParam(pos,
        static_cast<E_DjiCameraZoomDirection>(jsonInt(request_json, "direction", 0)),
        jsonDouble(request_json, "factor", 1.0));
  } else if (function == "get_optical_zoom") {
    T_DjiCameraManagerOpticalZoomParam zoom = {};
    rc = DjiCameraManager_GetOpticalZoomParam(pos, &zoom);
    std::ostringstream ss;
    ss << "{\"current\":" << zoom.currentOpticalZoomFactor << ",\"max\":" << zoom.maxOpticalZoomFactor << "}";
    json = ss.str();
  } else if (function == "set_infrared_zoom") {
    rc = DjiCameraManager_SetInfraredZoomParam(pos, jsonDouble(request_json, "zoom_factor", 1.0));
  } else if (function == "start_record_video") {
    rc = DjiCameraManager_StartRecordVideo(pos);
  } else if (function == "stop_record_video") {
    rc = DjiCameraManager_StopRecordVideo(pos);
  } else if (function == "format_storage") {
    if (!dangerousAllowed(request_json)) return {false, kUnsupported, "{}", "format_storage requires safety_confirm=true or allow_dangerous_commands"};
    rc = DjiCameraManager_FormatStorage(pos);
  } else if (function == "get_storage_info") {
    T_DjiCameraManagerStorageInfo info = {};
    rc = DjiCameraManager_GetStorageInfo(pos, &info);
    std::ostringstream ss;
    ss << "{\"total_capacity_mb\":" << info.totalCapacity << ",\"remain_capacity_mb\":" << info.remainCapacity << "}";
    json = ss.str();
  } else if (function == "get_laser_ranging_info") {
    T_DjiCameraManagerLaserRangingInfo info = {};
    rc = DjiCameraManager_GetLaserRangingInfo(pos, &info);
    std::ostringstream ss;
    ss << "{\"longitude\":" << info.longitude << ",\"latitude\":" << info.latitude
       << ",\"altitude\":" << info.altitude << ",\"distance\":" << info.distance
       << ",\"screen_x\":" << info.screenX << ",\"screen_y\":" << info.screenY
       << ",\"enable_lidar\":" << boolJson(info.enable_lidar) << ",\"exception\":" << static_cast<int>(info.exception) << "}";
    json = ss.str();
  } else if (function == "download_file_by_index") {
    rc = DjiCameraManager_DownloadFileByIndex(pos, static_cast<uint32_t>(jsonInt(request_json, "file_index", 0)));
  } else if (function == "delete_file_by_index") {
    if (!dangerousAllowed(request_json)) return {false, kUnsupported, "{}", "delete_file_by_index requires safety_confirm=true or allow_dangerous_commands"};
    rc = DjiCameraManager_DeleteFileByIndex(pos, static_cast<uint32_t>(jsonInt(request_json, "file_index", 0)));
  }

  if (rc != kUnsupported) {
    publishJson("camera", function, json);
    return {rc == kSuccess, rc, json, "camera_manager " + function};
  }
  return {false, kUnsupported, "{}", "unsupported camera_manager function"};
}

// Flight-controller commands convert JSON values into DJI enum/struct types.
// Takeoff, landing, motor, and emergency operations require the safety gate.
PsdkRos1Wrapper::ApiResult PsdkRos1Wrapper::dispatchFlightController(const std::string& function,
                                                                     const std::string& request_json)
{
  T_DjiReturnCode rc = kUnsupported;
  std::string json = "{}";
  if (function == "set_go_home_altitude") {
    rc = DjiFlightController_SetGoHomeAltitude(static_cast<E_DjiFlightControllerGoHomeAltitude>(jsonInt(request_json, "altitude", 50)));
  } else if (function == "get_go_home_altitude") {
    E_DjiFlightControllerGoHomeAltitude altitude;
    rc = DjiFlightController_GetGoHomeAltitude(&altitude);
    json = numberJson("altitude", static_cast<int>(altitude));
  } else if (function == "set_home_gps") {
    T_DjiFlightControllerHomeLocation home = {};
    home.latitude = jsonDouble(request_json, "latitude", 0.0);
    home.longitude = jsonDouble(request_json, "longitude", 0.0);
    rc = DjiFlightController_SetHomeLocationUsingGPSCoordinates(home);
  } else if (function == "set_home_current") {
    rc = DjiFlightController_SetHomeLocationUsingCurrentAircraftLocation();
  } else if (function == "obtain_joystick_authority") {
    rc = DjiFlightController_ObtainJoystickCtrlAuthority();
  } else if (function == "release_joystick_authority") {
    rc = DjiFlightController_ReleaseJoystickCtrlAuthority();
  } else if (function == "joystick") {
    T_DjiFlightControllerJoystickCommand cmd = {};
    cmd.x = jsonDouble(request_json, "x", 0.0);
    cmd.y = jsonDouble(request_json, "y", 0.0);
    cmd.z = jsonDouble(request_json, "z", 0.0);
    cmd.yaw = jsonDouble(request_json, "yaw", 0.0);
    rc = DjiFlightController_ExecuteJoystickAction(cmd);
  } else if (function == "start_go_home") {
    rc = DjiFlightController_StartGoHome();
  } else if (function == "cancel_go_home") {
    rc = DjiFlightController_CancelGoHome();
  } else if (function == "takeoff" || function == "landing" || function == "force_landing" ||
             function == "turn_on_motors" || function == "turn_off_motors" || function == "emergency_brake") {
    if (!dangerousAllowed(request_json)) return {false, kUnsupported, "{}", function + " requires safety_confirm=true or allow_dangerous_commands"};
    if (function == "takeoff") rc = DjiFlightController_StartTakeoff();
    else if (function == "landing") rc = DjiFlightController_StartLanding();
    else if (function == "force_landing") rc = DjiFlightController_StartForceLanding();
    else if (function == "turn_on_motors") rc = DjiFlightController_TurnOnMotors();
    else if (function == "turn_off_motors") rc = DjiFlightController_TurnOffMotors();
    else if (function == "emergency_brake") rc = DjiFlightController_ExecuteEmergencyBrakeAction();
  } else if (function == "set_horizontal_visual_obstacle_avoidance") {
    const E_DjiFlightControllerObstacleAvoidanceEnableStatus status =
        jsonBool(request_json, "enabled", true) ? DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE
                                                  : DJI_FLIGHT_CONTROLLER_DISABLE_OBSTACLE_AVOIDANCE;
    rc = DjiFlightController_SetHorizontalVisualObstacleAvoidanceEnableStatus(status);
  } else if (function == "get_horizontal_visual_obstacle_avoidance") {
    E_DjiFlightControllerObstacleAvoidanceEnableStatus status =
        DJI_FLIGHT_CONTROLLER_DISABLE_OBSTACLE_AVOIDANCE;
    rc = DjiFlightController_GetHorizontalVisualObstacleAvoidanceEnableStatus(&status);
    const bool enabled = status == DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE;
    json = std::string("{\"enabled\":") + boolJson(enabled) + "}";
  }
  if (rc != kUnsupported) {
    publishJson("flight_controller", function, json);
    return {rc == kSuccess, rc, json, "flight_controller " + function};
  }
  return {false, kUnsupported, "{}", "unsupported flight_controller function"};
}

// Gimbal manager commands are intentionally kept separate from the feedback
// subscription path so command results and measured state remain distinguishable.
PsdkRos1Wrapper::ApiResult PsdkRos1Wrapper::dispatchGimbalManager(const std::string& function,
                                                                  const std::string& request_json)
{
  E_DjiMountPosition pos = static_cast<E_DjiMountPosition>(jsonInt(request_json, "payload_index", 1));
  T_DjiReturnCode rc = kUnsupported;
  if (function == "set_mode") {
    rc = DjiGimbalManager_SetMode(pos, static_cast<E_DjiGimbalMode>(jsonInt(request_json, "mode", 0)));
  } else if (function == "reset") {
    rc = DjiGimbalManager_Reset(pos, static_cast<E_DjiGimbalResetMode>(jsonInt(request_json, "reset_mode", 1)));
  } else if (function == "rotate") {
    T_DjiGimbalManagerRotation rotation = {};
    rotation.rotationMode = static_cast<E_DjiGimbalRotationMode>(jsonInt(request_json, "rotation_mode", 0));
    rotation.pitch = jsonDouble(request_json, "pitch_deg", 0.0);
    rotation.roll = jsonDouble(request_json, "roll_deg", 0.0);
    rotation.yaw = jsonDouble(request_json, "yaw_deg", 0.0);
    rotation.time = jsonDouble(request_json, "time", 1.0);
    rc = DjiGimbalManager_Rotate(pos, rotation);
  } else if (function == "restore_factory_settings") {
    if (!dangerousAllowed(request_json)) return {false, kUnsupported, "{}", "restore_factory_settings requires safety_confirm=true or allow_dangerous_commands"};
    rc = DjiGimbalManager_RestoreFactorySettings(pos);
  }
  if (rc != kUnsupported) return {rc == kSuccess, rc, resultJson(rc), "gimbal_manager " + function};
  return {false, kUnsupported, "{}", "unsupported gimbal_manager function"};
}

// Liveview optionally forwards H.264 callback buffers to /raw; stopping a
// stream also disables raw publication to prevent stale callback forwarding.
PsdkRos1Wrapper::ApiResult PsdkRos1Wrapper::dispatchLiveview(const std::string& function, const std::string& request_json)
{
  E_DjiLiveViewCameraPosition pos = static_cast<E_DjiLiveViewCameraPosition>(jsonInt(request_json, "camera_position", 0));
  E_DjiLiveViewCameraSource source = static_cast<E_DjiLiveViewCameraSource>(jsonInt(request_json, "camera_source", 0));
  T_DjiReturnCode rc = kUnsupported;
  if (function == "start_h264_stream") {
    g_liveview_raw_publish_enabled = jsonBool(request_json, "publish_raw", false);
    rc = DjiLiveview_StartH264Stream(pos, source,
                                     g_liveview_raw_publish_enabled ? liveviewH264Callback : nullptr);
  } else if (function == "stop_h264_stream") {
    rc = DjiLiveview_StopH264Stream(pos, source);
    g_liveview_raw_publish_enabled = false;
  } else if (function == "request_intraframe") {
    rc = DjiLiveview_RequestIntraframeFrameData(pos, source);
  } else if (function == "unregister_encoder_callback") {
    rc = DjiLiveview_UnregEncoderCallback();
  }
  if (rc != kUnsupported) return {rc == kSuccess, rc, resultJson(rc), "liveview " + function};
  return {false, kUnsupported, "{}", "unsupported liveview function"};
}

// Perception currently exposes selected read/unsubscribe operations; raw
// perception payload expansion can be added without changing dispatchApi().
PsdkRos1Wrapper::ApiResult PsdkRos1Wrapper::dispatchPerception(const std::string& function, const std::string& request_json)
{
  T_DjiReturnCode rc = kUnsupported;
  if (function == "get_stereo_camera_parameters") {
    T_DjiPerceptionCameraParametersPacket packet = {};
    rc = DjiPerception_GetStereoCameraParameters(&packet);
    return {rc == kSuccess, rc, resultJson(rc), "stereo camera parameters requested; raw struct publication can be extended"};
  } else if (function == "unsubscribe_lidar") {
    rc = DjiPerception_UnsubscribeLidarData();
  } else if (function == "unsubscribe_radar") {
    rc = DjiPerception_UnsubscribeRadarData(static_cast<E_DjiPerceptionRadarPosition>(jsonInt(request_json, "position", 0)));
  } else if (function == "unsubscribe_image") {
    rc = DjiPerception_UnsubscribePerceptionImage(static_cast<E_DjiPerceptionDirection>(jsonInt(request_json, "direction", 0)));
  }
  if (rc != kUnsupported) return {rc == kSuccess, rc, resultJson(rc), "perception " + function};
  return {false, kUnsupported, "{}", "unsupported perception function"};
}

// Data-channel families share a JSON boundary because their payloads vary by
// module; empty cloud text payloads are rejected before entering the SDK.
PsdkRos1Wrapper::ApiResult PsdkRos1Wrapper::dispatchDataChannel(const std::string& module, const std::string& function,
                                                                const std::string& request_json)
{
  T_DjiReturnCode rc = kUnsupported;
  std::string json = "{}";
  if (module == "high_speed_data_channel") {
    if (function == "get_stream_state") {
      T_DjiDataChannelState state;
      rc = DjiHighSpeedDataChannel_GetDataStreamState(&state);
      std::ostringstream ss;
      ss << "{\"realtime_bandwidth_before_flow_controller\":"
         << state.realtimeBandwidthBeforeFlowController
         << ",\"realtime_bandwidth_after_flow_controller\":"
         << state.realtimeBandwidthAfterFlowController
         << ",\"busy_state\":" << boolJson(state.busyState) << "}";
      json = ss.str();
    } else if (function == "set_bandwidth") {
      T_DjiDataChannelBandwidthProportionOfHighspeedChannel bandwidth = {};
      bandwidth.dataStream = static_cast<uint8_t>(jsonInt(request_json, "data_stream", 33));
      bandwidth.videoStream = static_cast<uint8_t>(jsonInt(request_json, "video_stream", 33));
      bandwidth.downloadStream = static_cast<uint8_t>(jsonInt(request_json, "download_stream", 34));
      rc = DjiHighSpeedDataChannel_SetBandwidthProportion(bandwidth);
    }
  } else if (module == "low_speed_data_channel") {
    if (function == "get_send_state") {
      T_DjiDataChannelState state;
      rc = DjiLowSpeedDataChannel_GetSendDataState(static_cast<E_DjiChannelAddress>(jsonInt(request_json, "address", 0)), &state);
      std::ostringstream ss;
      ss << "{\"realtime_bandwidth_before_flow_controller\":"
         << state.realtimeBandwidthBeforeFlowController
         << ",\"realtime_bandwidth_after_flow_controller\":"
         << state.realtimeBandwidthAfterFlowController
         << ",\"busy_state\":" << boolJson(state.busyState) << "}";
      json = ss.str();
    }
  } else if (module == "cloud_api" && function == "send_text") {
    std::string payload = jsonString(request_json, "data", "");
    if (payload.empty()) {
      return {false, DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER, "{}", "cloud_api send_text requires non-empty data"};
    }
    uint32_t real_len = 0;
    rc = DjiCloudApi_SendDataByWebSocket(reinterpret_cast<uint8_t*>(&payload[0]), payload.size(), &real_len);
    json = numberJson("real_len", real_len);
  }
  if (rc != kUnsupported) return {rc == kSuccess, rc, json, module + " " + function};
  return {false, kUnsupported, "{}", "unsupported data channel function"};
}

// Waypoint V2/V3 and interest-point functions are grouped because their public
// ROS request shape is the same module/function/JSON contract.
PsdkRos1Wrapper::ApiResult PsdkRos1Wrapper::dispatchWaypoint(const std::string& module, const std::string& function,
                                                             const std::string& request_json)
{
  T_DjiReturnCode rc = kUnsupported;
  std::string json = "{}";
  if (module == "waypoint_v2") {
    if (function == "start") rc = DjiWaypointV2_Start();
    else if (function == "stop") rc = DjiWaypointV2_Stop();
    else if (function == "pause") rc = DjiWaypointV2_Pause();
    else if (function == "resume") rc = DjiWaypointV2_Resume();
    else if (function == "set_global_cruise_speed") rc = DjiWaypointV2_SetGlobalCruiseSpeed(jsonDouble(request_json, "speed", 1.0));
    else if (function == "get_global_cruise_speed") {
      T_DjiWaypointV2GlobalCruiseSpeed speed = 0;
      rc = DjiWaypointV2_GetGlobalCruiseSpeed(&speed);
      json = numberJson("speed", speed);
    }
  } else if (module == "waypoint_v3") {
    if (function == "action") rc = DjiWaypointV3_Action(static_cast<E_DjiWaypointV3Action>(jsonInt(request_json, "action", 0)));
  } else if (module == "interest_point") {
    if (function == "stop") rc = DjiInterestPoint_Stop();
    else if (function == "set_speed") rc = DjiInterestPoint_SetSpeed(jsonDouble(request_json, "speed", 1.0));
  }
  if (rc != kUnsupported) return {rc == kSuccess, rc, json, module + " " + function};
  return {false, kUnsupported, "{}", "unsupported waypoint/interest_point function"};
}

// Remaining utility modules are handled here, with power-output operations
// protected by dangerousAllowed() before any DJI command is issued.
PsdkRos1Wrapper::ApiResult PsdkRos1Wrapper::dispatchPowerAndUtility(const std::string& module, const std::string& function,
                                                                    const std::string& request_json)
{
  T_DjiReturnCode rc = kUnsupported;
  if (module == "power_management") {
    if (!dangerousAllowed(request_json)) return {false, kUnsupported, "{}", "power_management commands require safety_confirm=true or allow_dangerous_commands"};
    if (function == "apply_high_power") rc = DjiPowerManagement_ApplyHighPowerSync();
    else if (function == "apply_high_power_v2") rc = DjiPowerManagement_ApplyHighPowerSyncV2(static_cast<E_DjiHighPowerVoltage>(jsonInt(request_json, "voltage", 0)));
    else if (function == "output_high_power") rc = DjiPowerManagement_OutputHighPower(jsonBool(request_json, "enabled", false));
  } else if (module == "hms_customization") {
    if (function == "inject") rc = DjiHmsCustomization_InjectHmsErrorCode(jsonInt(request_json, "error_code", 0), static_cast<E_DjiHmsErrorLevel>(jsonInt(request_json, "level", 0)));
    else if (function == "eliminate") rc = DjiHmsCustomization_EliminateHmsErrorCode(jsonInt(request_json, "error_code", 0));
  } else if (module == "tethered_battery" && function == "push_tether_line_status") {
    T_DjiTetherLineStatus status = {};
    status.totalLength = jsonDouble(request_json, "total_length", 0.0);
    status.usedLength = jsonDouble(request_json, "used_length", 0.0);
    rc = DjiTetheredBattery_PushTetherLineStatus(status);
  } else if (module == "widget" && function == "show_message") {
    rc = DjiWidgetFloatingWindow_ShowMessage(jsonString(request_json, "message", "").c_str());
  } else if (module == "xport") {
    if (function == "set_gimbal_mode") rc = DjiXPort_SetGimbalModeSync(static_cast<E_DjiGimbalMode>(jsonInt(request_json, "mode", 0)));
    else if (function == "release_control") rc = DjiXPort_ReleaseControlPermissionSync();
    else if (function == "reset") rc = DjiXPort_ResetSync(static_cast<E_DjiGimbalResetMode>(jsonInt(request_json, "reset_mode", 1)));
    else if (function == "set_speed_conversion_factor") rc = DjiXPort_SetSpeedConversionFactor(jsonDouble(request_json, "factor", 1.0));
  } else if (module == "open_ar") {
    if (function == "clear_point") rc = DjiLiveview_ArClearPoint(static_cast<uint32_t>(jsonInt(request_json, "resource_id", 0)));
    else if (function == "clear_line") rc = DjiLiveview_ArClearLine();
    else if (function == "clear_polygon") rc = DjiLiveview_ArClearPolygon();
    else if (function == "clear_circle") rc = DjiLiveview_ArClearCircle();
    else if (function == "clear_pivot_axis") rc = DjiLiveview_ArClearPivotAxis();
  }
  if (rc != kUnsupported) return {rc == kSuccess, rc, resultJson(rc), module + " " + function};
  return {false, kUnsupported, "{}", "unsupported module/function"};
}

// State/result/event publishers intentionally keep the ROS observability layer
// separate from the PSDK call sites above.
void PsdkRos1Wrapper::publishModuleState(const std::string& module, bool initialized, bool available,
                                         T_DjiReturnCode return_code, const std::string& state,
                                         const std::string& message)
{
  PsdkModuleState msg;
  msg.header.stamp = ros::Time::now();
  msg.module = module;
  msg.initialized = initialized;
  msg.available = available;
  msg.return_code = return_code;
  msg.state = state;
  msg.message = message;
  module_state_pub_.publish(msg);
}

// Every API request is mirrored with its original module/function/request JSON
// so an external recorder can reconstruct the command/result sequence.
void PsdkRos1Wrapper::publishApiResult(const std::string& module, const std::string& function,
                                       const std::string& request_json, const ApiResult& result)
{
  PsdkApiResult msg;
  msg.header.stamp = ros::Time::now();
  msg.module = module;
  msg.function = function;
  msg.request_json = request_json;
  msg.success = result.success;
  msg.return_code = result.return_code;
  msg.response_json = result.response_json;
  msg.message = result.message;
  api_result_pub_.publish(msg);
}

// Events carry asynchronous module notifications and mission events that do
// not have a direct service response.
void PsdkRos1Wrapper::publishEvent(const std::string& module, const std::string& event_name,
                                   T_DjiReturnCode return_code, const std::string& payload_json,
                                   const std::string& message)
{
  PsdkEvent msg;
  msg.header.stamp = ros::Time::now();
  msg.module = module;
  msg.event_name = event_name;
  msg.return_code = return_code;
  msg.payload_json = payload_json;
  msg.message = message;
  event_pub_.publish(msg);
}

// publishJson is the generic read/event output for dispatcher branches that do
// not have a dedicated ROS message type.
void PsdkRos1Wrapper::publishJson(const std::string& module, const std::string& name, const std::string& json)
{
  PsdkJson msg;
  msg.header.stamp = ros::Time::now();
  msg.module = module;
  msg.name = name;
  msg.json = json;
  json_pub_.publish(msg);
}

// Core state is split into a human-readable state topic and a numeric DJI
// return-code topic so operators can consume either form independently.
void PsdkRos1Wrapper::publishCoreState(const std::string& state, T_DjiReturnCode return_code, const std::string& message)
{
  std_msgs::String state_msg;
  state_msg.data = state + ": " + message;
  core_state_pub_.publish(state_msg);
  std_msgs::Int64 code_msg;
  code_msg.data = return_code;
  core_return_code_pub_.publish(code_msg);
}

// Planner state reports both authority and freshness; a timeout is observable
// even though the timer sends a zero-hold command to the aircraft.
void PsdkRos1Wrapper::publishPlannerState(uint8_t state, bool authority, bool fresh,
                                          const std::string& message, T_DjiReturnCode return_code)
{
  PlannerState msg;
  msg.header.stamp = ros::Time::now();
  msg.state = state;
  msg.control_authority = authority;
  msg.command_fresh = fresh;
  msg.message = message;
  msg.return_code = return_code;
  planner_state_pub_.publish(msg);
}

// Mission state reports cached/uploaded data and the DJI result without
// exposing the raw KMZ bytes on the state topic.
void PsdkRos1Wrapper::publishMissionState(uint8_t state, const std::string& mission_id, uint32_t waypoint_count,
                                          float cruise_speed, const std::string& message, T_DjiReturnCode return_code)
{
  MissionState msg;
  msg.header.stamp = ros::Time::now();
  msg.state = state;
  msg.mission_id = mission_id;
  msg.waypoint_count = waypoint_count;
  msg.cruise_speed = cruise_speed;
  msg.message = message;
  msg.return_code = return_code;
  mission_state_pub_.publish(msg);
}

// Periodic aircraft status is gated on core initialization so no PSDK reads
// occur while the platform is only partially configured.
void PsdkRos1Wrapper::aircraftInfoTimerCb(const ros::TimerEvent&)
{
  if (!core_initialized_) {
    return;
  }
  dispatchAircraftInfo("get_connection_status", "{}");
}

// Translate the ROS planner mode message into DJI joystick control enums.
void PsdkRos1Wrapper::configureJoystickMode()
{
  T_DjiFlightControllerJoystickMode mode = {};
  if (planner_control_mode_.horizontal_mode == PlannerControlMode::HORIZONTAL_POSITION) {
    mode.horizontalControlMode = DJI_FLIGHT_CONTROLLER_HORIZONTAL_POSITION_CONTROL_MODE;
  } else if (planner_control_mode_.horizontal_mode == PlannerControlMode::HORIZONTAL_ANGLE) {
    mode.horizontalControlMode = DJI_FLIGHT_CONTROLLER_HORIZONTAL_ANGLE_CONTROL_MODE;
  } else {
    mode.horizontalControlMode = DJI_FLIGHT_CONTROLLER_HORIZONTAL_VELOCITY_CONTROL_MODE;
  }

  mode.verticalControlMode = planner_control_mode_.vertical_mode == PlannerControlMode::VERTICAL_POSITION
      ? DJI_FLIGHT_CONTROLLER_VERTICAL_POSITION_CONTROL_MODE
      : DJI_FLIGHT_CONTROLLER_VERTICAL_VELOCITY_CONTROL_MODE;
  mode.yawControlMode = planner_control_mode_.yaw_mode == PlannerControlMode::YAW_ANGLE
      ? DJI_FLIGHT_CONTROLLER_YAW_ANGLE_CONTROL_MODE
      : DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE;
  mode.horizontalCoordinate = planner_control_mode_.frame == PlannerControlMode::FRAME_GROUND
      ? DJI_FLIGHT_CONTROLLER_HORIZONTAL_GROUND_COORDINATE
      : DJI_FLIGHT_CONTROLLER_HORIZONTAL_BODY_COORDINATE;
  mode.stableControlMode = planner_control_mode_.stable_mode
      ? DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE
      : DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_DISABLE;
  DjiFlightController_SetJoystickMode(mode);
}

// Convert the newest planner command into DJI units and clamp it to configured
// velocity/yaw limits.  A zero command is used for timeout and disable paths.
T_DjiFlightControllerJoystickCommand PsdkRos1Wrapper::buildJoystickCommand(bool zero) const
{
  T_DjiFlightControllerJoystickCommand cmd = {};
  if (zero) {
    return cmd;
  }

  if (planner_has_velocity_cmd_) {
    cmd.x = clampValue(latest_velocity_cmd_.twist.linear.x, -params_.planner_max_horizontal_velocity, params_.planner_max_horizontal_velocity);
    cmd.y = clampValue(latest_velocity_cmd_.twist.linear.y, -params_.planner_max_horizontal_velocity, params_.planner_max_horizontal_velocity);
    cmd.z = clampValue(latest_velocity_cmd_.twist.linear.z, -params_.planner_max_vertical_velocity, params_.planner_max_vertical_velocity);
    cmd.yaw = clampValue(latest_velocity_cmd_.twist.angular.z * kRadToDeg, -params_.planner_max_yaw_rate, params_.planner_max_yaw_rate);
  } else if (planner_has_pose_cmd_) {
    cmd.x = latest_pose_cmd_.pose.position.x;
    cmd.y = latest_pose_cmd_.pose.position.y;
    cmd.z = latest_pose_cmd_.pose.position.z;
    cmd.yaw = yawFromQuaternion(latest_pose_cmd_.pose.orientation) * kRadToDeg;
  }
  return cmd;
}

double PsdkRos1Wrapper::clampValue(double value, double min_value, double max_value) const
{
  return std::max(min_value, std::min(max_value, value));
}

double PsdkRos1Wrapper::yawFromQuaternion(const geometry_msgs::Quaternion& q) const
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

// KMZ path input is restricted to configured roots before any file is opened.
// Raw KMZ bytes received through the KmzFile topic do not use this path check.
bool PsdkRos1Wrapper::isKmzPathAllowed(const std::string& path) const
{
  for (const std::string& root : params_.mission_allow_kmz_file_path_roots) {
    if (!root.empty() && path.find(root) == 0) {
      return true;
    }
  }
  return false;
}

// These parsers are deliberately conservative: missing, malformed, or
// non-convertible fields fall back to the caller-provided default value.
int PsdkRos1Wrapper::jsonInt(const std::string& json, const std::string& key, int default_value) const
{
  const std::string needle = "\"" + key + "\"";
  size_t pos = json.find(needle);
  if (pos == std::string::npos) return default_value;
  pos = json.find(':', pos);
  if (pos == std::string::npos) return default_value;
  try { return std::stoi(json.substr(pos + 1)); } catch (...) { return default_value; }
}

double PsdkRos1Wrapper::jsonDouble(const std::string& json, const std::string& key, double default_value) const
{
  const std::string needle = "\"" + key + "\"";
  size_t pos = json.find(needle);
  if (pos == std::string::npos) return default_value;
  pos = json.find(':', pos);
  if (pos == std::string::npos) return default_value;
  try { return std::stod(json.substr(pos + 1)); } catch (...) { return default_value; }
}

bool PsdkRos1Wrapper::jsonBool(const std::string& json, const std::string& key, bool default_value) const
{
  const std::string needle = "\"" + key + "\"";
  size_t pos = json.find(needle);
  if (pos == std::string::npos) return default_value;
  pos = json.find(':', pos);
  if (pos == std::string::npos) return default_value;
  std::string value = json.substr(pos + 1, 8);
  std::transform(value.begin(), value.end(), value.begin(), ::tolower);
  if (value.find("true") != std::string::npos || value.find("1") != std::string::npos) return true;
  if (value.find("false") != std::string::npos || value.find("0") != std::string::npos) return false;
  return default_value;
}

std::string PsdkRos1Wrapper::jsonString(const std::string& json, const std::string& key, const std::string& default_value) const
{
  const std::string needle = "\"" + key + "\"";
  size_t pos = json.find(needle);
  if (pos == std::string::npos) return default_value;
  pos = json.find(':', pos);
  if (pos == std::string::npos) return default_value;
  size_t first = json.find('"', pos + 1);
  if (first == std::string::npos) return default_value;
  size_t second = json.find('"', first + 1);
  if (second == std::string::npos) return default_value;
  return json.substr(first + 1, second - first - 1);
}

std::string PsdkRos1Wrapper::escapeJson(const std::string& value) const
{
  std::string out;
  for (char c : value) {
    if (c == '"' || c == '\\') out.push_back('\\');
    out.push_back(c);
  }
  return out;
}

std::string PsdkRos1Wrapper::resultJson(T_DjiReturnCode return_code) const
{
  return numberJson("return_code", return_code);
}

std::string PsdkRos1Wrapper::boolJson(bool value) const
{
  return value ? "true" : "false";
}

// Dangerous operations are allowed only by the deployment-wide switch or an
// explicit per-request confirmation field; the default is always rejection.
bool PsdkRos1Wrapper::dangerousAllowed(const std::string& request_json) const
{
  return params_.allow_dangerous_commands || jsonBool(request_json, "safety_confirm", false);
}

}  // namespace psdk_ros1
