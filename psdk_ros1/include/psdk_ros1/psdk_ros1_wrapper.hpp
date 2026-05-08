#ifndef PSDK_ROS1_PSDK_ROS1_WRAPPER_HPP
#define PSDK_ROS1_PSDK_ROS1_WRAPPER_HPP

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

#include <map>
#include <string>
#include <vector>

#include <dji_aircraft_info.h>
#include <dji_camera_manager.h>
#include <dji_cloud_api_by_websockt.h>
#include <dji_core.h>
#include <dji_fc_subscription.h>
#include <dji_flight_controller.h>
#include <dji_gimbal.h>
#include <dji_gimbal_manager.h>
#include <dji_high_speed_data_channel.h>
#include <dji_hms_customization.h>
#include <dji_hms_manager.h>
#include <dji_interest_point.h>
#include <dji_liveview.h>
#include <dji_logger.h>
#include <dji_low_speed_data_channel.h>
#include <dji_mop_channel.h>
#include <dji_open_ar.h>
#include <dji_payload_camera.h>
#include <dji_perception.h>
#include <dji_platform.h>
#include <dji_positioning.h>
#include <dji_power_management.h>
#include <dji_tethered_battery.h>
#include <dji_time_sync.h>
#include <dji_typedef.h>
#include <dji_upgrade.h>
#include <dji_waypoint_v2.h>
#include <dji_waypoint_v3.h>
#include <dji_widget.h>
#include <dji_widget_manager.h>
#include <dji_xport.h>

#include <hal_network.h>
#include <hal_uart.h>
#include <hal_usb_bulk.h>
#include <osal.h>
#include <osal_fs.h>
#include <osal_socket.h>
#include <utils/dji_config_manager.h>

#include <psdk_ros1/PsdkApi.h>
#include <psdk_ros1/PsdkApiResult.h>
#include <psdk_ros1/PsdkEvent.h>
#include <psdk_ros1/PsdkJson.h>
#include <psdk_ros1/PsdkModuleState.h>
#include <psdk_ros1/PsdkRawData.h>
#include <psdk_ros1/CameraFormatSdCard.h>
#include <psdk_ros1/CameraGetType.h>
#include <psdk_ros1/CameraRecordVideo.h>
#include <psdk_ros1/CameraSetupStreaming.h>
#include <psdk_ros1/CameraShootSinglePhoto.h>
#include <psdk_ros1/GetGoHomeAltitude.h>
#include <psdk_ros1/GimbalAngles.h>
#include <psdk_ros1/GimbalMode.h>
#include <psdk_ros1/GimbalReset.h>
#include <psdk_ros1/GimbalResetCmd.h>
#include <psdk_ros1/GimbalRotation.h>
#include <psdk_ros1/GimbalSetMode.h>
#include <psdk_ros1/GimbalStatus.h>
#include <psdk_ros1/JoystickCommand.h>
#include <psdk_ros1/KmzFile.h>
#include <psdk_ros1/MissionCommand.h>
#include <psdk_ros1/MissionState.h>
#include <psdk_ros1/PlannerControlMode.h>
#include <psdk_ros1/PlannerState.h>
#include <psdk_ros1/SetGoHomeAltitude.h>

namespace psdk_ros1
{

class PsdkRos1Wrapper
{
public:
  PsdkRos1Wrapper(ros::NodeHandle nh, ros::NodeHandle private_nh);
  ~PsdkRos1Wrapper();

  bool start();
  void shutdown();

private:
  struct Params
  {
    std::string app_name;
    std::string app_id;
    std::string app_key;
    std::string app_license;
    std::string developer_account;
    std::string baudrate;
    std::string link_config_file_path;
    std::string alias;
    std::string serial_number;
    std::vector<int> firmware_version;
    bool auto_start_modules;
    bool allow_dangerous_commands;
    int core_init_retries;
    double planner_control_rate;
    double planner_command_timeout;
    std::string planner_default_frame;
    bool planner_auto_obtain_authority;
    bool planner_release_authority_on_disable;
    double planner_max_horizontal_velocity;
    double planner_max_vertical_velocity;
    double planner_max_yaw_rate;
    int gimbal_feedback_payload_index;
    int gimbal_feedback_frequency;
    double mission_default_cruise_speed;
    double mission_reference_latitude;
    double mission_reference_longitude;
    double mission_reference_altitude;
    bool mission_reference_configured;
    std::vector<std::string> mission_allow_kmz_file_path_roots;
  };

  struct ApiResult
  {
    bool success;
    T_DjiReturnCode return_code;
    std::string response_json;
    std::string message;
  };

  bool loadParams();
  bool registerPlatformHandlers();
  bool fillUserInfo(T_DjiUserInfo* user_info) const;
  bool initCore();
  bool initDefaultModules();
  bool deinitDefaultModules();
  bool subscribeGimbalFeedback();
  void unsubscribeGimbalFeedback();
  E_DjiFcSubscriptionTopic gimbalAnglesTopicForPayload(uint8_t payload_index) const;

  bool handleInitialize(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool handleShutdown(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool handleApi(PsdkApi::Request& req, PsdkApi::Response& res);
  bool handleCameraFormatSdCard(CameraFormatSdCard::Request& req, CameraFormatSdCard::Response& res);
  bool handleCameraGetType(CameraGetType::Request& req, CameraGetType::Response& res);
  bool handleCameraRecordVideo(CameraRecordVideo::Request& req, CameraRecordVideo::Response& res);
  bool handleCameraSetupStreaming(CameraSetupStreaming::Request& req, CameraSetupStreaming::Response& res);
  bool handleCameraShootSinglePhoto(CameraShootSinglePhoto::Request& req, CameraShootSinglePhoto::Response& res);
  bool handleGimbalSetMode(GimbalSetMode::Request& req, GimbalSetMode::Response& res);
  bool handleGimbalReset(GimbalReset::Request& req, GimbalReset::Response& res);
  bool handleSetGoHomeAltitude(SetGoHomeAltitude::Request& req, SetGoHomeAltitude::Response& res);
  bool handleGetGoHomeAltitude(GetGoHomeAltitude::Request& req, GetGoHomeAltitude::Response& res);

  void gimbalRotationCb(const GimbalRotation::ConstPtr& msg);
  void gimbalModeCb(const GimbalMode::ConstPtr& msg);
  void gimbalResetCb(const GimbalResetCmd::ConstPtr& msg);
  void plannerVelocityCb(const geometry_msgs::TwistStamped::ConstPtr& msg);
  void plannerPoseCb(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void plannerEnableCb(const std_msgs::Bool::ConstPtr& msg);
  void plannerEmergencyStopCb(const std_msgs::Bool::ConstPtr& msg);
  void plannerControlModeCb(const PlannerControlMode::ConstPtr& msg);
  void plannerTimerCb(const ros::TimerEvent& event);
  void missionPathCb(const nav_msgs::Path::ConstPtr& msg);
  void missionCommandCb(const MissionCommand::ConstPtr& msg);
  void missionKmzFileCb(const KmzFile::ConstPtr& msg);
  void missionKmzPathCb(const std_msgs::String::ConstPtr& msg);

  ApiResult dispatchApi(const std::string& module, const std::string& function, const std::string& request_json);
  ApiResult dispatchCore(const std::string& function, const std::string& request_json);
  ApiResult dispatchAircraftInfo(const std::string& function, const std::string& request_json);
  ApiResult dispatchModuleLifecycle(const std::string& module, const std::string& function, const std::string& request_json);
  ApiResult dispatchCameraManager(const std::string& function, const std::string& request_json);
  ApiResult dispatchFlightController(const std::string& function, const std::string& request_json);
  ApiResult dispatchGimbalManager(const std::string& function, const std::string& request_json);
  ApiResult dispatchLiveview(const std::string& function, const std::string& request_json);
  ApiResult dispatchPerception(const std::string& function, const std::string& request_json);
  ApiResult dispatchDataChannel(const std::string& module, const std::string& function, const std::string& request_json);
  ApiResult dispatchWaypoint(const std::string& module, const std::string& function, const std::string& request_json);
  ApiResult dispatchPowerAndUtility(const std::string& module, const std::string& function, const std::string& request_json);

  void publishModuleState(const std::string& module, bool initialized, bool available,
                          T_DjiReturnCode return_code, const std::string& state,
                          const std::string& message);
  void publishApiResult(const std::string& module, const std::string& function,
                        const std::string& request_json, const ApiResult& result);
  void publishEvent(const std::string& module, const std::string& event_name,
                    T_DjiReturnCode return_code, const std::string& payload_json,
                    const std::string& message);
  void publishJson(const std::string& module, const std::string& name, const std::string& json);
  void publishCoreState(const std::string& state, T_DjiReturnCode return_code, const std::string& message);
  void publishPlannerState(uint8_t state, bool authority, bool fresh, const std::string& message, T_DjiReturnCode return_code);
  void publishMissionState(uint8_t state, const std::string& mission_id, uint32_t waypoint_count,
                           float cruise_speed, const std::string& message, T_DjiReturnCode return_code);
  void aircraftInfoTimerCb(const ros::TimerEvent& event);
  void configureJoystickMode();
  T_DjiFlightControllerJoystickCommand buildJoystickCommand(bool zero) const;
  double clampValue(double value, double min_value, double max_value) const;
  double yawFromQuaternion(const geometry_msgs::Quaternion& q) const;
  bool isKmzPathAllowed(const std::string& path) const;

  int jsonInt(const std::string& json, const std::string& key, int default_value) const;
  double jsonDouble(const std::string& json, const std::string& key, double default_value) const;
  bool jsonBool(const std::string& json, const std::string& key, bool default_value) const;
  std::string jsonString(const std::string& json, const std::string& key, const std::string& default_value) const;
  std::string escapeJson(const std::string& value) const;
  std::string resultJson(T_DjiReturnCode return_code) const;
  std::string boolJson(bool value) const;
  bool dangerousAllowed(const std::string& request_json) const;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  Params params_;

  ros::Publisher core_state_pub_;
  ros::Publisher core_return_code_pub_;
  ros::Publisher module_state_pub_;
  ros::Publisher event_pub_;
  ros::Publisher raw_pub_;
  ros::Publisher api_result_pub_;
  ros::Publisher json_pub_;
  ros::Publisher gimbal_command_result_pub_;
  ros::Publisher gimbal_angles_pub_;
  ros::Publisher gimbal_status_pub_;
  ros::Publisher planner_state_pub_;
  ros::Publisher planner_command_result_pub_;
  ros::Publisher joystick_cmd_echo_pub_;
  ros::Publisher mission_state_pub_;
  ros::Publisher mission_event_pub_;

  ros::Subscriber gimbal_rotation_sub_;
  ros::Subscriber gimbal_mode_sub_;
  ros::Subscriber gimbal_reset_sub_;
  ros::Subscriber planner_velocity_sub_;
  ros::Subscriber planner_pose_sub_;
  ros::Subscriber planner_enable_sub_;
  ros::Subscriber planner_emergency_stop_sub_;
  ros::Subscriber planner_control_mode_sub_;
  ros::Subscriber mission_path_sub_;
  ros::Subscriber mission_command_sub_;
  ros::Subscriber mission_kmz_file_sub_;
  ros::Subscriber mission_kmz_path_sub_;

  ros::ServiceServer initialize_srv_;
  ros::ServiceServer shutdown_srv_;
  ros::ServiceServer api_srv_;
  ros::ServiceServer camera_format_sd_card_srv_;
  ros::ServiceServer camera_get_type_srv_;
  ros::ServiceServer camera_record_video_srv_;
  ros::ServiceServer camera_setup_streaming_srv_;
  ros::ServiceServer camera_shoot_single_photo_srv_;
  ros::ServiceServer gimbal_set_mode_srv_;
  ros::ServiceServer gimbal_reset_srv_;
  ros::ServiceServer set_go_home_altitude_srv_;
  ros::ServiceServer get_go_home_altitude_srv_;
  ros::Timer aircraft_info_timer_;
  ros::Timer planner_timer_;

  bool platform_ready_;
  bool core_initialized_;
  bool modules_initialized_;
  bool gimbal_angles_feedback_subscribed_;
  bool gimbal_status_feedback_subscribed_;
  bool planner_enabled_;
  bool planner_has_authority_;
  bool planner_emergency_stop_;
  bool planner_has_velocity_cmd_;
  bool planner_has_pose_cmd_;
  geometry_msgs::TwistStamped latest_velocity_cmd_;
  geometry_msgs::PoseStamped latest_pose_cmd_;
  ros::Time latest_planner_cmd_time_;
  PlannerControlMode planner_control_mode_;
  nav_msgs::Path latest_mission_path_;
  bool mission_has_path_;
  std::vector<uint8_t> latest_kmz_data_;
  std::string latest_kmz_file_name_;
  bool mission_has_kmz_;
  std::string latest_kmz_path_;
};

}  // namespace psdk_ros1

#endif  // PSDK_ROS1_PSDK_ROS1_WRAPPER_HPP
