#include <psdk_ros1/GimbalAngles.h>
#include <psdk_ros1/GimbalMode.h>
#include <psdk_ros1/GimbalRotation.h>
#include <psdk_ros1/GimbalStatus.h>
#include <psdk_ros1/PlannerControlMode.h>
#include <psdk_ros1/PsdkApi.h>
#include <psdk_ros1/TrackerState.h>

#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <vision_msgs/Detection2DArray.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

/**
 * ROS-side visual target tracker.  Detector observations are fused with
 * gimbal feedback, converted into bounded gimbal commands, and optionally
 * accompanied by short-lived aircraft yaw/ lateral assistance near gimbal
 * limits.  The node never calls DJI PSDK directly; it uses ROS topics and the
 * wrapper's generic API service.
 */
namespace psdk_ros1
{
namespace
{
// Angles are stored internally in radians for ROS/control math; configuration
// values and detector geometry remain in the units stated by the YAML params.
const double kPi = 3.14159265358979323846;
const double kDegToRad = kPi / 180.0;

double clampValue(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(max_value, value));
}

// Keep yaw commands in the principal interval before publishing them.
double wrapPi(double value)
{
  while (value > kPi) {
    value -= 2.0 * kPi;
  }
  while (value < -kPi) {
    value += 2.0 * kPi;
  }
  return value;
}

// Target-class filtering is optional; an empty list accepts every detector ID.
bool containsId(const std::vector<int>& ids, int id)
{
  return std::find(ids.begin(), ids.end(), id) != ids.end();
}

// Limit reasons are accumulated without duplicate labels for operator-facing
// TrackerState diagnostics.
void appendReason(std::string& reason, const std::string& item)
{
  if (reason.find(item) != std::string::npos) {
    return;
  }
  reason = reason.empty() ? item : reason + "," + item;
}

// Support both the ROS1 Noetic direct `id` field and compatible detector types
// exposing a string `class_id`, without changing the tracker contract.
template <typename Hypothesis>
auto hypothesisClassId(const Hypothesis& hypothesis, int) -> decltype(hypothesis.id, int())
{
  return static_cast<int>(hypothesis.id);
}

template <typename Hypothesis>
auto hypothesisClassId(const Hypothesis& hypothesis, long) -> decltype(hypothesis.class_id, int())
{
  try {
    return std::stoi(hypothesis.class_id);
  } catch (...) {
    return std::numeric_limits<int>::min();
  }
}

inline int hypothesisClassId(...)
{
  return std::numeric_limits<int>::min();
}
}  // namespace

// State machine: DISABLED -> SEARCH -> ACQUIRE -> TRACK, with RECOVER and
// LIMITED_LOST paths protecting the gimbal/planner when detections disappear
// or physical limits are reached.
class VisualTargetTracker
{
public:
  // Construction wires all ROS I/O and starts the fixed-rate control timer;
  // enabling motion remains an explicit service action.
  VisualTargetTracker(ros::NodeHandle nh, ros::NodeHandle private_nh)
    : nh_(nh), private_nh_(private_nh), enabled_(false), filter_initialized_(false),
      wide_valid_(false), zoom_valid_(false), has_gimbal_angles_(false), has_gimbal_status_(false),
      planner_enabled_by_tracker_(false), assist_active_(false), scan_index_(0), lock_count_(0),
      state_(TrackerState::DISABLED), last_loop_time_(ros::Time::now())
  {
    loadParams();

    wide_sub_ = nh_.subscribe(wide_detection_topic_, 5, &VisualTargetTracker::wideDetectionsCb, this);
    zoom_sub_ = nh_.subscribe(zoom_detection_topic_, 5, &VisualTargetTracker::zoomDetectionsCb, this);
    gimbal_angles_sub_ = nh_.subscribe("/psdk/gimbal/angles", 10, &VisualTargetTracker::gimbalAnglesCb, this);
    gimbal_status_sub_ = nh_.subscribe("/psdk/gimbal/status", 10, &VisualTargetTracker::gimbalStatusCb, this);

    gimbal_cmd_pub_ = nh_.advertise<GimbalRotation>("/psdk/gimbal/rotation_cmd", 10);
    gimbal_mode_pub_ = nh_.advertise<GimbalMode>("/psdk/gimbal/mode_cmd", 2, true);
    planner_mode_pub_ = nh_.advertise<PlannerControlMode>("/psdk/planner/control_mode", 2, true);
    planner_enable_pub_ = nh_.advertise<std_msgs::Bool>("/psdk/planner/enable", 2, true);
    planner_velocity_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/psdk/planner/velocity_cmd", 10);
    state_pub_ = nh_.advertise<TrackerState>("/psdk/tracker/state", 10, true);
    enable_srv_ = nh_.advertiseService("/psdk/tracker/enable", &VisualTargetTracker::handleEnable, this);
    api_client_ = nh_.serviceClient<PsdkApi>("/psdk/api/call");

    buildScanPattern();
    timer_ = nh_.createTimer(ros::Duration(1.0 / control_rate_), &VisualTargetTracker::timerCb, this);
  }

private:
  // An Observation is normalized detector geometry plus confidence and source
  // identity.  cx/cy/width/height use input image pixels; normalized errors
  // are computed only when the image dimensions are known.
  struct Observation
  {
    bool valid = false;
    uint8_t source = TrackerState::SOURCE_NONE;
    ros::Time stamp;
    double cx = 0.0;
    double cy = 0.0;
    double width = 0.0;
    double height = 0.0;
    double image_width = 1920.0;
    double image_height = 1080.0;
    double confidence = 0.0;
  };

  struct Pid
  {
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
    double integral = 0.0;
    double prev_error = 0.0;
    bool has_prev = false;

    // PID output is clamped to the caller's angular-rate limit; dt is seconds.
    double update(double error, double dt, double limit)
    {
      if (dt <= 0.0) {
        return clampValue(kp * error, -limit, limit);
      }
      integral = clampValue(integral + error * dt, -limit, limit);
      const double derivative = has_prev ? (error - prev_error) / dt : 0.0;
      prev_error = error;
      has_prev = true;
      return clampValue(kp * error + ki * integral + kd * derivative, -limit, limit);
    }

    void reset()
    {
      integral = 0.0;
      prev_error = 0.0;
      has_prev = false;
    }
  };

  class ConstantVelocityKalman
  {
  public:
    ConstantVelocityKalman()
    {
      reset();
    }

    void reset()
    {
      x_.fill(0.0);
      for (std::array<double, 4>& row : p_) {
        row.fill(0.0);
      }
      p_[0][0] = 0.05;
      p_[1][1] = 0.05;
      p_[2][2] = 1.0;
      p_[3][3] = 1.0;
    }

    // Four-state model: [normalized_x, normalized_y, velocity_x, velocity_y].
    // Covariance/process constants are tracker tuning values, not ROS params.
    void init(double x, double y)
    {
      reset();
      x_[0] = x;
      x_[1] = y;
      x_[2] = 0.0;
      x_[3] = 0.0;
    }

    void predict(double dt)
    {
      dt = clampValue(dt, 0.0, 0.5);
      x_[0] += x_[2] * dt;
      x_[1] += x_[3] * dt;

      std::array<std::array<double, 4>, 4> f = {};
      for (int i = 0; i < 4; ++i) {
        f[i][i] = 1.0;
      }
      f[0][2] = dt;
      f[1][3] = dt;

      std::array<std::array<double, 4>, 4> fp = {};
      std::array<std::array<double, 4>, 4> fpf_t = {};
      for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
          for (int k = 0; k < 4; ++k) {
            fp[r][c] += f[r][k] * p_[k][c];
          }
        }
      }
      for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
          for (int k = 0; k < 4; ++k) {
            fpf_t[r][c] += fp[r][k] * f[c][k];
          }
        }
      }
      p_ = fpf_t;

      const double q_pos = 0.0025 + 0.02 * dt * dt;
      const double q_vel = 0.05 + 0.1 * dt;
      p_[0][0] += q_pos;
      p_[1][1] += q_pos;
      p_[2][2] += q_vel;
      p_[3][3] += q_vel;
    }

    void update(double z_x, double z_y)
    {
      updateAxis(0, z_x);
      updateAxis(1, z_y);
    }

    double x() const { return x_[0]; }
    double y() const { return x_[1]; }

  private:
    void updateAxis(int axis, double z)
    {
      const double r = 0.025;
      const double y = z - x_[axis];
      const double s = p_[axis][axis] + r;
      if (s <= 1e-9) {
        return;
      }

      std::array<double, 4> k = {};
      for (int i = 0; i < 4; ++i) {
        k[i] = p_[i][axis] / s;
      }
      for (int i = 0; i < 4; ++i) {
        x_[i] += k[i] * y;
      }

      std::array<std::array<double, 4>, 4> new_p = p_;
      for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
          new_p[i][j] -= k[i] * p_[axis][j];
        }
      }
      p_ = new_p;
    }

    std::array<double, 4> x_;
    std::array<std::array<double, 4>, 4> p_;
  };

  // Load tracker geometry, rates, control limits, and PID/filter tuning from
  // the private namespace, then convert configured angular limits to radians.
  void loadParams()
  {
    private_nh_.param<int>("payload_index", payload_index_, 1);
    private_nh_.param<std::string>("wide_detection_topic", wide_detection_topic_, "/detector/h20_wide/detections");
    private_nh_.param<std::string>("zoom_detection_topic", zoom_detection_topic_, "/detector/h20_zoom/detections");
    private_nh_.param<double>("control_rate", control_rate_, 30.0);
    private_nh_.param<std::string>("assist_mode", assist_mode_, "yaw_only");
    private_nh_.param<double>("max_gimbal_yaw_rate_deg_s", max_gimbal_yaw_rate_deg_s_, 60.0);
    private_nh_.param<double>("max_gimbal_pitch_rate_deg_s", max_gimbal_pitch_rate_deg_s_, 45.0);
    private_nh_.param<double>("max_aircraft_yaw_rate_deg_s", max_aircraft_yaw_rate_deg_s_, 20.0);
    private_nh_.param<double>("max_horizontal_velocity_m_s", max_horizontal_velocity_m_s_, 0.5);
    private_nh_.param<double>("max_horizontal_accel_m_s2", max_horizontal_accel_m_s2_, 0.2);
    private_nh_.param<double>("detection_timeout_s", detection_timeout_s_, 0.25);
    private_nh_.param<double>("lost_to_search_timeout_s", lost_to_search_timeout_s_, 1.0);
    private_nh_.param<double>("detector_latency_s", detector_latency_s_, 0.08);
    private_nh_.param<int>("wide_image_width", wide_image_width_, 1920);
    private_nh_.param<int>("wide_image_height", wide_image_height_, 1080);
    private_nh_.param<int>("zoom_image_width", zoom_image_width_, 1920);
    private_nh_.param<int>("zoom_image_height", zoom_image_height_, 1080);
    private_nh_.param<double>("wide_deadzone", wide_deadzone_, 0.03);
    private_nh_.param<double>("zoom_deadzone", zoom_deadzone_, 0.015);
    private_nh_.param<double>("yaw_sign", yaw_sign_, 1.0);
    private_nh_.param<double>("pitch_sign", pitch_sign_, -1.0);
    private_nh_.param<double>("soft_yaw_limit_deg", soft_yaw_limit_deg_, 250.0);
    private_nh_.param<double>("soft_pitch_lower_deg", soft_pitch_lower_deg_, -105.0);
    private_nh_.param<double>("soft_pitch_upper_deg", soft_pitch_upper_deg_, 15.0);
    private_nh_.param<double>("hard_yaw_limit_deg", hard_yaw_limit_deg_, 300.0);
    private_nh_.param<double>("hard_pitch_lower_deg", hard_pitch_lower_deg_, -115.0);
    private_nh_.param<double>("hard_pitch_upper_deg", hard_pitch_upper_deg_, 25.0);
    private_nh_.param<double>("max_assist_duration_s", max_assist_duration_s_, 5.0);
    private_nh_.param<double>("acquire_error", acquire_error_, 0.20);
    private_nh_.param<int>("acquire_count", acquire_count_required_, 5);
    private_nh_.param<double>("search_home_pitch_deg", search_home_pitch_deg_, -20.0);
    private_nh_.param<double>("search_rotation_time_s", search_rotation_time_s_, 0.25);
    private_nh_.param<bool>("enable_zoom_control", enable_zoom_control_, false);
    private_nh_.param<double>("zoom_control_interval_s", zoom_control_interval_s_, 1.0);
    private_nh_.param<double>("zoom_target_bbox_height", zoom_target_bbox_height_, 0.25);
    private_nh_.param<double>("zoom_bbox_deadband", zoom_bbox_deadband_, 0.08);
    private_nh_.param<double>("zoom_step_factor", zoom_step_factor_, 1.0);
    private_nh_.param<double>("kp_yaw", yaw_pid_.kp, 0.85);
    private_nh_.param<double>("ki_yaw", yaw_pid_.ki, 0.0);
    private_nh_.param<double>("kd_yaw", yaw_pid_.kd, 0.12);
    private_nh_.param<double>("kp_pitch", pitch_pid_.kp, 0.75);
    private_nh_.param<double>("ki_pitch", pitch_pid_.ki, 0.0);
    private_nh_.param<double>("kd_pitch", pitch_pid_.kd, 0.10);
    private_nh_.param<std::vector<int> >("target_classes", target_class_ids_, std::vector<int>());

    control_rate_ = std::max(1.0, control_rate_);
    max_gimbal_yaw_rate_ = std::abs(max_gimbal_yaw_rate_deg_s_) * kDegToRad;
    max_gimbal_pitch_rate_ = std::abs(max_gimbal_pitch_rate_deg_s_) * kDegToRad;
    max_aircraft_yaw_rate_ = std::abs(max_aircraft_yaw_rate_deg_s_) * kDegToRad;
    if (assist_mode_ != "disabled" && assist_mode_ != "yaw_only" && assist_mode_ != "yaw_xy") {
      ROS_WARN_STREAM("Unknown assist_mode '" << assist_mode_ << "', falling back to yaw_only");
      assist_mode_ = "yaw_only";
    }
  }

  // Enabling resets stale filter/command state and enters SEARCH; disabling
  // sends zero gimbal/planner commands before publishing the disabled state.
  bool handleEnable(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
  {
    enabled_ = req.data;
    resetTracking();
    if (enabled_) {
      state_ = TrackerState::SEARCH;
      publishGimbalMode(0);
      publishAbsoluteGimbal(search_home_pitch_deg_ * kDegToRad, 0.0, 0.6);
      res.message = "visual target tracker enabled";
    } else {
      state_ = TrackerState::DISABLED;
      publishZeroGimbalSpeed();
      setPlannerEnabled(false);
      res.message = "visual target tracker disabled";
    }
    res.success = true;
    publishState(TrackerState::SOURCE_NONE, 0.0, 0.0, 0.0, 0.0, 0.0, "");
    return true;
  }

  // Reset all temporal state so a newly enabled tracker cannot reuse an old
  // detection, PID integral, filter estimate, or aircraft-assist command.
  void resetTracking()
  {
    filter_.reset();
    filter_initialized_ = false;
    wide_valid_ = false;
    zoom_valid_ = false;
    lock_count_ = 0;
    scan_index_ = 0;
    last_scan_time_ = ros::Time(0);
    last_detection_time_ = ros::Time(0);
    last_measurement_stamp_ = ros::Time(0);
    last_measurement_source_ = TrackerState::SOURCE_NONE;
    yaw_pid_.reset();
    pitch_pid_.reset();
    last_aircraft_y_cmd_ = 0.0;
    last_aircraft_yaw_rate_cmd_ = 0.0;
    assist_active_ = false;
    assist_start_time_ = ros::Time(0);
  }

  // Wide and zoom detector streams are reduced to one best observation each;
  // freshness and source preference are resolved later in chooseObservation().
  void wideDetectionsCb(const vision_msgs::Detection2DArray::ConstPtr& msg)
  {
    Observation obs;
    if (selectBestDetection(*msg, TrackerState::SOURCE_WIDE, wide_image_width_, wide_image_height_, obs)) {
      wide_obs_ = obs;
      wide_valid_ = true;
    }
  }

  void zoomDetectionsCb(const vision_msgs::Detection2DArray::ConstPtr& msg)
  {
    Observation obs;
    if (selectBestDetection(*msg, TrackerState::SOURCE_ZOOM, zoom_image_width_, zoom_image_height_, obs)) {
      zoom_obs_ = obs;
      zoom_valid_ = true;
    }
  }

  // Select the highest confidence detection after optional class filtering,
  // with a small predicted-distance penalty to reduce target switching.
  bool selectBestDetection(const vision_msgs::Detection2DArray& msg, uint8_t source, int default_width,
                           int default_height, Observation& out) const
  {
    double best_score = -std::numeric_limits<double>::infinity();
    Observation best;
    const ros::Time stamp = msg.header.stamp.isZero() ? ros::Time::now() : msg.header.stamp;
    for (const vision_msgs::Detection2D& detection : msg.detections) {
      if (detection.bbox.size_x <= 0.0 || detection.bbox.size_y <= 0.0 || detection.results.empty()) {
        continue;
      }

      double confidence = -std::numeric_limits<double>::infinity();
      for (const vision_msgs::ObjectHypothesisWithPose& result : detection.results) {
        const int class_id = hypothesisClassId(result, 0);
        if (!target_class_ids_.empty() && !containsId(target_class_ids_, class_id)) {
          continue;
        }
        confidence = std::max(confidence, result.score);
      }
      if (!std::isfinite(confidence)) {
        continue;
      }

      const double image_width = detection.source_img.width > 0 ? detection.source_img.width : default_width;
      const double image_height = detection.source_img.height > 0 ? detection.source_img.height : default_height;
      const double ex = normalizedErrorX(detection.bbox.center.x, image_width);
      const double ey = normalizedErrorY(detection.bbox.center.y, image_height);
      const double predicted_distance =
          filter_initialized_ ? std::hypot(ex - filter_.x(), ey - filter_.y()) : 0.0;
      const double score = confidence - 0.15 * predicted_distance;
      if (score > best_score) {
        best_score = score;
        best.valid = true;
        best.source = source;
        best.stamp = stamp;
        best.cx = detection.bbox.center.x;
        best.cy = detection.bbox.center.y;
        best.width = detection.bbox.size_x;
        best.height = detection.bbox.size_y;
        best.image_width = image_width;
        best.image_height = image_height;
        best.confidence = confidence;
      }
    }

    if (!best.valid) {
      return false;
    }
    out = best;
    return true;
  }

  // Only feedback for the configured payload slot is accepted; this prevents
  // another mounted gimbal from steering the tracker state.
  void gimbalAnglesCb(const GimbalAngles::ConstPtr& msg)
  {
    if (msg->payload_index != static_cast<uint8_t>(payload_index_)) {
      return;
    }
    gimbal_angles_ = *msg;
    has_gimbal_angles_ = true;
  }

  void gimbalStatusCb(const GimbalStatus::ConstPtr& msg)
  {
    gimbal_status_ = *msg;
    has_gimbal_status_ = true;
  }

  // Main state-machine tick.  It prefers fresh observations, predicts briefly
  // during a loss, then returns to SEARCH and disables flight assistance.
  void timerCb(const ros::TimerEvent&)
  {
    const ros::Time now = ros::Time::now();
    const double dt = clampValue((now - last_loop_time_).toSec(), 1.0 / (control_rate_ * 2.0), 0.2);
    last_loop_time_ = now;

    if (!enabled_) {
      publishState(TrackerState::SOURCE_NONE, 0.0, 0.0, 0.0, 0.0, 0.0, "");
      return;
    }

    Observation obs;
    if (chooseObservation(now, obs)) {
      updateWithObservation(obs, now, dt);
      return;
    }

    const bool recently_seen = !last_detection_time_.isZero() &&
                               (now - last_detection_time_).toSec() <= lost_to_search_timeout_s_ &&
                               filter_initialized_;
    if (recently_seen) {
      state_ = TrackerState::RECOVER;
      filter_.predict(dt);
      runServo(TrackerState::SOURCE_NONE, 0.0, filter_.x(), filter_.y(), dt, now);
      return;
    }

    state_ = TrackerState::SEARCH;
    filter_initialized_ = false;
    setPlannerEnabled(false);
    runSearch(now);
    publishState(TrackerState::SOURCE_NONE, 0.0, 0.0, 0.0, 0.0, 0.0, "");
  }

  // During ACQUIRE/TRACK prefer a fresh zoom observation; during SEARCH prefer
  // wide FoV so the tracker can reacquire a target before zooming in.
  bool chooseObservation(const ros::Time& now, Observation& out) const
  {
    const bool wide_fresh = wide_valid_ && (now - wide_obs_.stamp).toSec() <= detection_timeout_s_;
    const bool zoom_fresh = zoom_valid_ && (now - zoom_obs_.stamp).toSec() <= detection_timeout_s_;
    if (!wide_fresh && !zoom_fresh) {
      return false;
    }

    if (state_ == TrackerState::TRACK || state_ == TrackerState::ACQUIRE) {
      if (zoom_fresh) {
        out = zoom_obs_;
        return true;
      }
      out = wide_obs_;
      return true;
    }

    if (wide_fresh) {
      out = wide_obs_;
      return true;
    }
    out = zoom_obs_;
    return true;
  }

  // Fuse a new measurement or predict through a repeated/lost frame, then
  // transition to TRACK after the configured number of near-center locks.
  void updateWithObservation(const Observation& obs, const ros::Time& now, double dt)
  {
    const double error_x = normalizedErrorX(obs.cx, obs.image_width);
    const double error_y = normalizedErrorY(obs.cy, obs.image_height);
    const bool new_measurement = obs.stamp != last_measurement_stamp_ || obs.source != last_measurement_source_;

    if (new_measurement) {
      if (!filter_initialized_) {
        filter_.init(error_x, error_y);
        filter_initialized_ = true;
      } else {
        filter_.predict(std::max(0.0, (obs.stamp - last_measurement_stamp_).toSec()));
        filter_.update(error_x, error_y);
      }
      last_measurement_stamp_ = obs.stamp;
      last_measurement_source_ = obs.source;
      last_detection_time_ = now;
    } else if (filter_initialized_) {
      filter_.predict(dt);
    }
    if (std::hypot(error_x, error_y) < acquire_error_) {
      ++lock_count_;
    } else {
      lock_count_ = 0;
    }
    state_ = lock_count_ >= acquire_count_required_ ? TrackerState::TRACK : TrackerState::ACQUIRE;
    if (state_ == TrackerState::TRACK) {
      maybeUpdateZoom(obs, now);
    }

    filter_.predict(clampValue(detector_latency_s_, 0.0, 0.2));
    runServo(obs.source, obs.confidence, filter_.x(), filter_.y(), dt, now);
  }

  // Zoom is controlled through the wrapper API only after TRACK is stable and
  // the bounding-box height leaves the configured target/deadband interval.
  void maybeUpdateZoom(const Observation& obs, const ros::Time& now)
  {
    if (!enable_zoom_control_ || obs.image_height <= 1.0 ||
        (!last_zoom_time_.isZero() && (now - last_zoom_time_).toSec() < zoom_control_interval_s_)) {
      return;
    }

    const double normalized_height = obs.height / obs.image_height;
    int direction = -1;
    if (normalized_height < zoom_target_bbox_height_ - zoom_bbox_deadband_) {
      direction = 1;
    } else if (normalized_height > zoom_target_bbox_height_ + zoom_bbox_deadband_) {
      direction = 0;
    } else {
      return;
    }

    PsdkApi srv;
    srv.request.module = "camera_manager";
    srv.request.function = "set_optical_zoom";
    std::ostringstream json;
    json << "{\"payload_index\":" << payload_index_
         << ",\"direction\":" << direction
         << ",\"factor\":" << zoom_step_factor_ << "}";
    srv.request.request_json = json.str();
    last_zoom_time_ = now;
    api_client_.call(srv);
  }

  // PID steering is softened near limits, stopped at hard limits, and can
  // request bounded aircraft assistance when the selected assist mode allows it.
  void runServo(uint8_t source, double confidence, double error_x, double error_y, double dt, const ros::Time& now)
  {
    const double deadzone = source == TrackerState::SOURCE_ZOOM ? zoom_deadzone_ : wide_deadzone_;
    const double control_error_x = std::abs(error_x) < deadzone ? 0.0 : error_x;
    const double control_error_y = std::abs(error_y) < deadzone ? 0.0 : error_y;

    double yaw_cmd = yaw_sign_ * yaw_pid_.update(control_error_x, dt, max_gimbal_yaw_rate_);
    double pitch_cmd = pitch_sign_ * pitch_pid_.update(control_error_y, dt, max_gimbal_pitch_rate_);

    std::string limit_reason;
    const bool soft_limit = nearSoftLimit(limit_reason);
    const bool hard_limit = nearHardLimit(limit_reason);
    if (soft_limit) {
      yaw_cmd *= 0.45;
      pitch_cmd *= 0.45;
    }
    if (hard_limit) {
      yaw_cmd = 0.0;
      pitch_cmd = 0.0;
    }

    if (hard_limit && assist_mode_ == "disabled") {
      state_ = TrackerState::LIMITED_LOST;
      publishZeroGimbalSpeed();
      setPlannerEnabled(false);
      publishState(source, confidence, error_x, error_y, 0.0, 0.0, limit_reason);
      return;
    }

    publishGimbalSpeed(pitch_cmd, yaw_cmd);
    const bool request_assist = soft_limit && assist_mode_ != "disabled";
    runFlightAssist(request_assist, error_x, yaw_cmd, now, dt);
    publishState(source, confidence, error_x, error_y, yaw_cmd, pitch_cmd, limit_reason);
  }

  // Soft limits reduce command authority but still permit tracking; reasons are
  // retained for TrackerState diagnostics.
  bool nearSoftLimit(std::string& reason) const
  {
    bool limited = false;
    if (has_gimbal_status_) {
      if (gimbal_status_.yaw_limited) {
        appendReason(reason, "yaw_limited");
        limited = true;
      }
      if (gimbal_status_.pitch_limited) {
        appendReason(reason, "pitch_limited");
        limited = true;
      }
      if (gimbal_status_.roll_limited) {
        appendReason(reason, "roll_limited");
        limited = true;
      }
    }
    if (has_gimbal_angles_) {
      if (std::abs(gimbal_angles_.yaw) >= soft_yaw_limit_deg_ * kDegToRad) {
        appendReason(reason, "yaw_soft_limit");
        limited = true;
      }
      if (gimbal_angles_.pitch <= soft_pitch_lower_deg_ * kDegToRad ||
          gimbal_angles_.pitch >= soft_pitch_upper_deg_ * kDegToRad) {
        appendReason(reason, "pitch_soft_limit");
        limited = true;
      }
    }
    return limited;
  }

  // Hard limits zero the gimbal command and, when necessary, force LIMITED_LOST
  // instead of continuing to push against a physical boundary.
  bool nearHardLimit(std::string& reason) const
  {
    bool limited = false;
    if (has_gimbal_status_) {
      if (gimbal_status_.yaw_limited) {
        appendReason(reason, "yaw_limited");
        limited = true;
      }
      if (gimbal_status_.pitch_limited) {
        appendReason(reason, "pitch_limited");
        limited = true;
      }
      if (gimbal_status_.roll_limited) {
        appendReason(reason, "roll_limited");
        limited = true;
      }
    }
    if (has_gimbal_angles_) {
      if (std::abs(gimbal_angles_.yaw) >= hard_yaw_limit_deg_ * kDegToRad) {
        appendReason(reason, "yaw_hard_limit");
        limited = true;
      }
      if (gimbal_angles_.pitch <= hard_pitch_lower_deg_ * kDegToRad ||
          gimbal_angles_.pitch >= hard_pitch_upper_deg_ * kDegToRad) {
        appendReason(reason, "pitch_hard_limit");
        limited = true;
      }
    }
    return limited;
  }

  // Flight assistance is time-bounded.  It converts image x error into yaw
  // rate and optional body-y velocity, with acceleration limiting on y.
  void runFlightAssist(bool request_assist, double error_x, double gimbal_yaw_cmd, const ros::Time& now, double dt)
  {
    if (!request_assist) {
      assist_active_ = false;
      setPlannerEnabled(false);
      return;
    }

    if (!assist_active_) {
      assist_active_ = true;
      assist_start_time_ = now;
    }
    if ((now - assist_start_time_).toSec() > max_assist_duration_s_) {
      setPlannerEnabled(false);
      return;
    }

    setPlannerEnabled(true);
    geometry_msgs::TwistStamped cmd;
    cmd.header.stamp = now;
    last_aircraft_yaw_rate_cmd_ = clampValue(gimbal_yaw_cmd + yaw_sign_ * error_x * max_aircraft_yaw_rate_,
                                             -max_aircraft_yaw_rate_, max_aircraft_yaw_rate_);
    cmd.twist.angular.z = last_aircraft_yaw_rate_cmd_;

    double target_y = 0.0;
    if (assist_mode_ == "yaw_xy") {
      target_y = clampValue(yaw_sign_ * error_x * max_horizontal_velocity_m_s_,
                            -max_horizontal_velocity_m_s_, max_horizontal_velocity_m_s_);
    }
    const double max_delta = std::abs(max_horizontal_accel_m_s2_) * dt;
    last_aircraft_y_cmd_ += clampValue(target_y - last_aircraft_y_cmd_, -max_delta, max_delta);
    cmd.twist.linear.y = last_aircraft_y_cmd_;
    planner_velocity_pub_.publish(cmd);
  }

  // Search visits a bounded pitch/yaw pattern only after the lost timeout; the
  // dwell prevents detector latency from causing excessively rapid sweeps.
  void runSearch(const ros::Time& now)
  {
    const double dwell = std::max(2.0 * detector_latency_s_, 0.12);
    if (!last_scan_time_.isZero() && (now - last_scan_time_).toSec() < dwell) {
      return;
    }
    if (scan_pattern_.empty()) {
      return;
    }

    const std::pair<double, double>& point = scan_pattern_[scan_index_ % scan_pattern_.size()];
    publishAbsoluteGimbal(point.first, point.second, search_rotation_time_s_);
    last_scan_time_ = now;
    ++scan_index_;
  }

  // Build the deterministic scan order once from configured home pitch and
  // hard limits; published commands are converted to radians below.
  void buildScanPattern()
  {
    scan_pattern_.clear();
    const std::vector<double> yaw_deg = {0.0, 15.0, -15.0, 35.0, -35.0, 70.0, -70.0,
                                         120.0, -120.0, 180.0, -180.0, 240.0, -240.0};
    const std::vector<double> pitch_deg = {search_home_pitch_deg_, -45.0, 0.0, -70.0, 10.0};
    for (double pitch : pitch_deg) {
      for (double yaw : yaw_deg) {
        scan_pattern_.push_back(std::make_pair(clampValue(pitch, hard_pitch_lower_deg_, hard_pitch_upper_deg_) * kDegToRad,
                                               clampValue(yaw, -hard_yaw_limit_deg_, hard_yaw_limit_deg_) * kDegToRad));
      }
    }
  }

  double normalizedErrorX(double cx, double width) const
  {
    return width > 1.0 ? (cx - width * 0.5) / (width * 0.5) : 0.0;
  }

  double normalizedErrorY(double cy, double height) const
  {
    return height > 1.0 ? (cy - height * 0.5) / (height * 0.5) : 0.0;
  }

  // rotation_mode=1 is absolute and rotation_mode=2 is rate control in the
  // package message contract; both paths publish radians and bounded duration.
  void publishGimbalMode(uint8_t mode)
  {
    GimbalMode msg;
    msg.header.stamp = ros::Time::now();
    msg.payload_index = static_cast<uint8_t>(payload_index_);
    msg.gimbal_mode = mode;
    gimbal_mode_pub_.publish(msg);
  }

  void publishAbsoluteGimbal(double pitch, double yaw, double time)
  {
    GimbalRotation msg;
    msg.header.stamp = ros::Time::now();
    msg.payload_index = static_cast<uint8_t>(payload_index_);
    msg.rotation_mode = 1;
    msg.pitch = static_cast<float>(pitch);
    msg.roll = 0.0f;
    msg.yaw = static_cast<float>(wrapPi(yaw));
    msg.time = static_cast<float>(time);
    gimbal_cmd_pub_.publish(msg);
  }

  void publishGimbalSpeed(double pitch_rate, double yaw_rate)
  {
    GimbalRotation msg;
    msg.header.stamp = ros::Time::now();
    msg.payload_index = static_cast<uint8_t>(payload_index_);
    msg.rotation_mode = 2;
    msg.pitch = static_cast<float>(clampValue(pitch_rate, -max_gimbal_pitch_rate_, max_gimbal_pitch_rate_));
    msg.roll = 0.0f;
    msg.yaw = static_cast<float>(clampValue(yaw_rate, -max_gimbal_yaw_rate_, max_gimbal_yaw_rate_));
    msg.time = static_cast<float>(1.0 / control_rate_);
    gimbal_cmd_pub_.publish(msg);
  }

  void publishZeroGimbalSpeed()
  {
    publishGimbalSpeed(0.0, 0.0);
  }

  // Tracker-owned planner enable/disable is edge-triggered.  Disabling emits a
  // zero TwistStamped and clears the local assist command accumulators.
  void setPlannerEnabled(bool enabled)
  {
    if (enabled == planner_enabled_by_tracker_) {
      if (!enabled) {
        last_aircraft_y_cmd_ = 0.0;
        last_aircraft_yaw_rate_cmd_ = 0.0;
      }
      return;
    }

    if (enabled) {
      PlannerControlMode mode;
      mode.header.stamp = ros::Time::now();
      mode.horizontal_mode = PlannerControlMode::HORIZONTAL_VELOCITY;
      mode.vertical_mode = PlannerControlMode::VERTICAL_VELOCITY;
      mode.yaw_mode = PlannerControlMode::YAW_RATE;
      mode.frame = PlannerControlMode::FRAME_BODY;
      mode.stable_mode = true;
      planner_mode_pub_.publish(mode);
    } else {
      geometry_msgs::TwistStamped zero;
      zero.header.stamp = ros::Time::now();
      planner_velocity_pub_.publish(zero);
      last_aircraft_y_cmd_ = 0.0;
      last_aircraft_yaw_rate_cmd_ = 0.0;
    }

    std_msgs::Bool msg;
    msg.data = enabled;
    planner_enable_pub_.publish(msg);
    planner_enabled_by_tracker_ = enabled;
  }

  // Publish one complete control snapshot so downstream nodes can distinguish
  // detector confidence, image error, gimbal command, limits, and assistance.
  void publishState(uint8_t source, double confidence, double error_x, double error_y,
                    double yaw_cmd, double pitch_cmd, const std::string& limit_reason)
  {
    TrackerState msg;
    msg.header.stamp = ros::Time::now();
    msg.state = enabled_ ? state_ : TrackerState::DISABLED;
    msg.source = source;
    msg.assist_mode = assist_mode_;
    msg.confidence = static_cast<float>(confidence);
    msg.error_x = static_cast<float>(error_x);
    msg.error_y = static_cast<float>(error_y);
    msg.gimbal_yaw_cmd = static_cast<float>(yaw_cmd);
    msg.gimbal_pitch_cmd = static_cast<float>(pitch_cmd);
    msg.aircraft_yaw_rate_cmd = static_cast<float>(last_aircraft_yaw_rate_cmd_);
    msg.aircraft_x_cmd = 0.0f;
    msg.aircraft_y_cmd = static_cast<float>(last_aircraft_y_cmd_);
    msg.gimbal_limit_active = !limit_reason.empty();
    msg.flight_assist_active = planner_enabled_by_tracker_;
    msg.limit_reason = limit_reason;
    state_pub_.publish(msg);
  }

  // ROS handles and command/output endpoints.
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber wide_sub_;
  ros::Subscriber zoom_sub_;
  ros::Subscriber gimbal_angles_sub_;
  ros::Subscriber gimbal_status_sub_;
  ros::Publisher gimbal_cmd_pub_;
  ros::Publisher gimbal_mode_pub_;
  ros::Publisher planner_mode_pub_;
  ros::Publisher planner_enable_pub_;
  ros::Publisher planner_velocity_pub_;
  ros::Publisher state_pub_;
  ros::ServiceClient api_client_;
  ros::ServiceServer enable_srv_;
  ros::Timer timer_;

  // Configuration values: *_deg_* are degrees-based parameters; converted rate
  // limits below are radians/s, while velocities/accelerations are SI units.
  int payload_index_;
  std::string wide_detection_topic_;
  std::string zoom_detection_topic_;
  std::string assist_mode_;
  std::vector<int> target_class_ids_;
  double control_rate_;
  double max_gimbal_yaw_rate_deg_s_;
  double max_gimbal_pitch_rate_deg_s_;
  double max_aircraft_yaw_rate_deg_s_;
  double max_horizontal_velocity_m_s_;
  double max_horizontal_accel_m_s2_;
  double max_gimbal_yaw_rate_;
  double max_gimbal_pitch_rate_;
  double max_aircraft_yaw_rate_;
  double detection_timeout_s_;
  double lost_to_search_timeout_s_;
  double detector_latency_s_;
  int wide_image_width_;
  int wide_image_height_;
  int zoom_image_width_;
  int zoom_image_height_;
  double wide_deadzone_;
  double zoom_deadzone_;
  double yaw_sign_;
  double pitch_sign_;
  double soft_yaw_limit_deg_;
  double soft_pitch_lower_deg_;
  double soft_pitch_upper_deg_;
  double hard_yaw_limit_deg_;
  double hard_pitch_lower_deg_;
  double hard_pitch_upper_deg_;
  double max_assist_duration_s_;
  double acquire_error_;
  int acquire_count_required_;
  double search_home_pitch_deg_;
  double search_rotation_time_s_;
  bool enable_zoom_control_;
  double zoom_control_interval_s_;
  double zoom_target_bbox_height_;
  double zoom_bbox_deadband_;
  double zoom_step_factor_;

  // Runtime state shared by subscriber callbacks and the single ROS timer.
  bool enabled_;
  bool filter_initialized_;
  bool wide_valid_;
  bool zoom_valid_;
  bool has_gimbal_angles_;
  bool has_gimbal_status_;
  bool planner_enabled_by_tracker_;
  bool assist_active_;
  size_t scan_index_;
  int lock_count_;
  uint8_t state_;
  double last_aircraft_y_cmd_ = 0.0;
  double last_aircraft_yaw_rate_cmd_ = 0.0;
  Observation wide_obs_;
  Observation zoom_obs_;
  ConstantVelocityKalman filter_;
  Pid yaw_pid_;
  Pid pitch_pid_;
  GimbalAngles gimbal_angles_;
  GimbalStatus gimbal_status_;
  ros::Time last_loop_time_;
  ros::Time last_detection_time_;
  ros::Time last_measurement_stamp_;
  uint8_t last_measurement_source_;
  ros::Time last_scan_time_;
  ros::Time last_zoom_time_;
  ros::Time assist_start_time_;
  std::vector<std::pair<double, double> > scan_pattern_;
};

}  // namespace psdk_ros1

// Standalone ROS node for the tracker; the PSDK wrapper remains a separate
// process/node that owns the actual gimbal/planner service implementation.
int main(int argc, char** argv)
{
  ros::init(argc, argv, "psdk_visual_target_tracker_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  psdk_ros1::VisualTargetTracker tracker(nh, private_nh);
  ros::spin();
  return 0;
}
