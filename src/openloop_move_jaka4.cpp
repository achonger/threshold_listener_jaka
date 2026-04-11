#include <algorithm>
#include <cmath>
#include <cctype>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <jaka_msgs/Move.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

class OpenloopMoveJaka4
{
public:
  OpenloopMoveJaka4()
    : pnh_("~")
  {
    pnh_.param("total_distance_m", total_distance_m_, 0.10);
    pnh_.param("step_distance_m", step_distance_m_, 0.01);
    pnh_.param("total_distance_mm", total_distance_mm_, -1.0);
    pnh_.param("step_distance_mm", step_distance_mm_, -1.0);
    pnh_.param("small_test_step_mm", small_test_step_mm_, 50.0);

    pnh_.param("dwell_sec", dwell_sec_, 10.0);
    pnh_.param("command_settle_sec", command_settle_sec_, 1.0);
    pnh_.param("wait_pose_timeout_sec", wait_pose_timeout_sec_, 8.0);
    pnh_.param("wait_motion_start_timeout_sec", wait_motion_start_timeout_sec_, 3.0);
    pnh_.param("wait_motion_done_timeout_sec", wait_motion_done_timeout_sec_, 20.0);
    pnh_.param("motion_joint_threshold_rad", motion_joint_threshold_rad_, 0.01);
    pnh_.param("motion_stable_duration_sec", motion_stable_duration_sec_, 0.8);

    pnh_.param<std::string>("arm_ns", arm_ns_, "jaka4");
    pnh_.param<std::string>("tool_pose_unit", tool_pose_unit_, "mm");
    pnh_.param<std::string>("debug_motion_mode", debug_motion_mode_, "single_step");

    const std::string default_tool_pose_topic = "/" + arm_ns_ + "/jaka_driver/tool_position";
    const std::string default_linear_move_service = "/" + arm_ns_ + "/jaka_driver/linear_move";
    const std::string default_joint_states_topic = "/" + arm_ns_ + "/joint_states";
    pnh_.param<std::string>("tool_pose_topic", tool_pose_topic_, default_tool_pose_topic);
    pnh_.param<std::string>("linear_move_service", linear_move_service_, default_linear_move_service);
    pnh_.param<std::string>("joint_states_topic", joint_states_topic_, default_joint_states_topic);

    pnh_.param("mvvelo", mvvelo_, 30.0);
    pnh_.param("mvacc", mvacc_, 30.0);
    pnh_.param("mvtime", mvtime_, 0.0);
    pnh_.param("mvradii", mvradii_, 0.0);
    pnh_.param("coord_mode", coord_mode_, 0);
    pnh_.param("index", index_, 0);

    pose_sub_ = nh_.subscribe(tool_pose_topic_, 1, &OpenloopMoveJaka4::poseCallback, this);
    joint_sub_ = nh_.subscribe(joint_states_topic_, 20, &OpenloopMoveJaka4::jointCallback, this);
    linear_move_client_ = nh_.serviceClient<jaka_msgs::Move>(linear_move_service_);
  }

  int run()
  {
    ROS_INFO("[openloop_move_jaka4] pose topic: %s", tool_pose_topic_.c_str());
    ROS_INFO("[openloop_move_jaka4] joint_states topic: %s", joint_states_topic_.c_str());
    ROS_INFO("[openloop_move_jaka4] linear_move service: %s", linear_move_service_.c_str());
    ROS_INFO("[openloop_move_jaka4] tool_pose_unit: %s", tool_pose_unit_.c_str());
    ROS_INFO("[openloop_move_jaka4] debug_motion_mode: %s", debug_motion_mode_.c_str());
    ROS_INFO("[openloop_move_jaka4] motion thresholds: start_timeout=%.2f done_timeout=%.2f joint_threshold=%.6f stable_duration=%.2f",
             wait_motion_start_timeout_sec_, wait_motion_done_timeout_sec_,
             motion_joint_threshold_rad_, motion_stable_duration_sec_);

    const bool use_mm_params = (total_distance_mm_ > 0.0 && step_distance_mm_ > 0.0);
    if (use_mm_params) {
      active_total_mm_ = total_distance_mm_;
      active_step_mm_ = step_distance_mm_;
      ROS_INFO("[openloop_move_jaka4] Using mm params: total_distance_mm=%.3f, step_distance_mm=%.3f",
               active_total_mm_, active_step_mm_);
    } else {
      active_total_mm_ = total_distance_m_ * 1000.0;
      active_step_mm_ = step_distance_m_ * 1000.0;
      ROS_INFO("[openloop_move_jaka4] Using legacy m params: total_distance_m=%.6f, step_distance_m=%.6f -> total_mm=%.3f, step_mm=%.3f",
               total_distance_m_, step_distance_m_, active_total_mm_, active_step_mm_);
    }

    if (!linear_move_client_.waitForExistence(ros::Duration(wait_pose_timeout_sec_))) {
      ROS_ERROR("[openloop_move_jaka4] Service unavailable within %.2f sec: %s",
                wait_pose_timeout_sec_, linear_move_service_.c_str());
      return 1;
    }

    if (!waitForInitialData()) {
      if (has_pose_ && !has_joint_state_) {
        ROS_ERROR("[openloop_move_jaka4] Initial tool pose received, but no joint_states on %s within %.2f sec. Exit.",
                  joint_states_topic_.c_str(), wait_pose_timeout_sec_);
      } else if (!has_pose_ && has_joint_state_) {
        ROS_ERROR("[openloop_move_jaka4] joint_states received, but no tool pose on %s within %.2f sec. Exit.",
                  tool_pose_topic_.c_str(), wait_pose_timeout_sec_);
      } else {
        ROS_ERROR("[openloop_move_jaka4] Failed to get initial pose/joint_states within %.2f sec. Exit.", wait_pose_timeout_sec_);
      }
      return 1;
    }

    const int max_steps = static_cast<int>(active_total_mm_ / active_step_mm_ + 1e-6);
    if (max_steps <= 0) {
      ROS_ERROR("[openloop_move_jaka4] Invalid step setup. active_total_mm=%.3f, active_step_mm=%.3f",
                active_total_mm_, active_step_mm_);
      return 1;
    }

    if (debug_motion_mode_ == "hold") {
      ROS_INFO("[openloop_move_jaka4] Mode=hold, send current pose without movement.");
      return sendStep(initial_pose_, 0, false);
    }

    if (debug_motion_mode_ == "single_step") {
      geometry_msgs::PoseStamped target = initial_pose_;
      target.pose.position.x = initial_pose_.pose.position.x - toTopicDeltaFromMm(small_test_step_mm_);

      const double current_x_mm = topicPosToMm(initial_pose_.pose.position.x);
      const double target_x_mm = topicPosToMm(target.pose.position.x);
      const double delta_x_mm = target_x_mm - current_x_mm;

      ROS_INFO("[openloop_move_jaka4] Mode=single_step, move negative X by %.3f mm.", small_test_step_mm_);
      ROS_INFO("[openloop_move_jaka4] single_step details: current_x_mm=%.3f target_x_mm=%.3f delta_x_mm=%.3f",
               current_x_mm, target_x_mm, delta_x_mm);
      return sendStep(target, 1, true);
    }

    if (debug_motion_mode_ != "multi_step") {
      ROS_ERROR("[openloop_move_jaka4] Unknown debug_motion_mode=%s. Use hold/single_step/multi_step.",
                debug_motion_mode_.c_str());
      return 1;
    }

    ROS_INFO("[openloop_move_jaka4] Mode=multi_step, steps=%d, step=%.3f mm, total=%.3f mm",
             max_steps, active_step_mm_, active_total_mm_);

    for (int i = 1; i <= max_steps && ros::ok(); ++i) {
      geometry_msgs::PoseStamped target = initial_pose_;
      const double delta_mm = active_step_mm_ * static_cast<double>(i);
      target.pose.position.x = initial_pose_.pose.position.x - toTopicDeltaFromMm(delta_mm);

      if (sendStep(target, i, true) != 0) {
        return 1;
      }

      ros::Duration(command_settle_sec_).sleep();
      ROS_INFO("[openloop_move_jaka4] Step %d dwell %.2f sec.", i, dwell_sec_);
      ros::Duration(dwell_sec_).sleep();
    }

    ROS_INFO("[openloop_move_jaka4] Completed multi_step total %.3f mm displacement.", active_total_mm_);
    return 0;
  }

private:
  int sendStep(const geometry_msgs::PoseStamped& target, int step_idx, bool expect_motion)
  {
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Quaternion q(
      target.pose.orientation.x,
      target.pose.orientation.y,
      target.pose.orientation.z,
      target.pose.orientation.w);
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    const double x_mm = topicPosToMm(target.pose.position.x);
    const double y_mm = topicPosToMm(target.pose.position.y);
    const double z_mm = topicPosToMm(target.pose.position.z);

    ROS_INFO("[openloop_move_jaka4] Step %d raw topic pose: x=%.6f y=%.6f z=%.6f q=[%.6f %.6f %.6f %.6f]",
             step_idx,
             target.pose.position.x, target.pose.position.y, target.pose.position.z,
             target.pose.orientation.x, target.pose.orientation.y,
             target.pose.orientation.z, target.pose.orientation.w);
    ROS_INFO("[openloop_move_jaka4] Step %d converted RPY(rad): roll=%.6f pitch=%.6f yaw=%.6f",
             step_idx, roll, pitch, yaw);
    ROS_INFO("[openloop_move_jaka4] Step %d unit=%s -> x_mm=%.3f y_mm=%.3f z_mm=%.3f",
             step_idx, tool_pose_unit_.c_str(), x_mm, y_mm, z_mm);

    std::vector<double> joint_before = latest_joint_positions_;

    jaka_msgs::Move srv;
    srv.request.pose.clear();
    srv.request.pose.push_back(static_cast<float>(x_mm));
    srv.request.pose.push_back(static_cast<float>(y_mm));
    srv.request.pose.push_back(static_cast<float>(z_mm));
    srv.request.pose.push_back(static_cast<float>(roll));
    srv.request.pose.push_back(static_cast<float>(pitch));
    srv.request.pose.push_back(static_cast<float>(yaw));

    srv.request.has_ref = false;
    srv.request.ref_joint.clear();
    srv.request.mvvelo = mvvelo_;
    srv.request.mvacc = mvacc_;
    srv.request.mvtime = mvtime_;
    srv.request.mvradii = mvradii_;
    srv.request.coord_mode = coord_mode_;
    srv.request.index = index_;

    ROS_INFO("[openloop_move_jaka4] Step %d request.pose=[%.3f, %.3f, %.3f, %.6f, %.6f, %.6f]",
             step_idx,
             srv.request.pose[0], srv.request.pose[1], srv.request.pose[2],
             srv.request.pose[3], srv.request.pose[4], srv.request.pose[5]);
    ROS_INFO("[openloop_move_jaka4] Step %d params: mvvelo=%.3f mvacc=%.3f mvtime=%.3f mvradii=%.3f coord_mode=%d index=%d",
             step_idx, mvvelo_, mvacc_, mvtime_, mvradii_, coord_mode_, index_);

    // Stage 1: command send.
    if (!linear_move_client_.call(srv)) {
      ROS_ERROR("[openloop_move_jaka4] Step %d service call failed (ROS communication failure).", step_idx);
      return 1;
    }
    ROS_INFO("[openloop_move_jaka4] Step %d linear_move service call succeeded.", step_idx);

    const bool response_warning = isSuspiciousDriverResponse(srv.response.ret, srv.response.message);
    if (response_warning) {
      ROS_WARN("[openloop_move_jaka4] Step %d driver response indicates warning: ret=%d, message=%s",
               step_idx, srv.response.ret, srv.response.message.c_str());
    } else {
      ROS_INFO("[openloop_move_jaka4] Step %d driver response: ret=%d, message=%s",
               step_idx, srv.response.ret, srv.response.message.c_str());
    }

    // Stage 2: motion observation. Do not fail immediately on ERR_FUCTION_CALL_ERROR.
    if (expect_motion) {
      bool moved = false;
      bool settled = false;
      if (!waitMotionStart(joint_before, moved)) {
        ROS_ERROR("[openloop_move_jaka4] Step %d no effective joint motion within timeout.", step_idx);
        return 1;
      }
      if (!waitMotionSettle(settled)) {
        ROS_ERROR("[openloop_move_jaka4] Step %d motion did not settle in time.", step_idx);
        return 1;
      }
      ROS_INFO("[openloop_move_jaka4] Step %d motion settled, treat command as successful%s.",
               step_idx,
               response_warning ? " despite warning" : "");
    }

    return 0;
  }

  bool waitMotionStart(const std::vector<double>& baseline, bool& moved)
  {
    moved = false;
    ros::Time start = ros::Time::now();
    ros::Rate rate(100.0);
    while (ros::ok()) {
      ros::spinOnce();
      if (has_joint_state_) {
        const double diff = maxJointDiff(baseline, latest_joint_positions_);
        if (diff > motion_joint_threshold_rad_) {
          moved = true;
          ROS_INFO("[openloop_move_jaka4] Joint motion detected, max_diff=%.6f rad", diff);
          return true;
        }
      }
      if ((ros::Time::now() - start).toSec() > wait_motion_start_timeout_sec_) {
        return false;
      }
      rate.sleep();
    }
    return false;
  }

  bool waitMotionSettle(bool& settled)
  {
    settled = false;
    ros::Time start = ros::Time::now();
    ros::Time stable_since = ros::Time(0);
    std::vector<double> last_joint = latest_joint_positions_;
    ros::Rate rate(100.0);

    while (ros::ok()) {
      ros::spinOnce();
      if (has_joint_state_) {
        const double diff = maxJointDiff(last_joint, latest_joint_positions_);
        if (diff < motion_joint_threshold_rad_ * 0.5) {
          if (stable_since.isZero()) {
            stable_since = ros::Time::now();
          }
          if ((ros::Time::now() - stable_since).toSec() >= motion_stable_duration_sec_) {
            settled = true;
            return true;
          }
        } else {
          stable_since = ros::Time(0);
        }
        last_joint = latest_joint_positions_;
      }

      if ((ros::Time::now() - start).toSec() > wait_motion_done_timeout_sec_) {
        return false;
      }
      rate.sleep();
    }

    return false;
  }

  static double maxJointDiff(const std::vector<double>& a, const std::vector<double>& b)
  {
    const size_t n = std::min(a.size(), b.size());
    double d = 0.0;
    for (size_t i = 0; i < n; ++i) {
      d = std::max(d, std::fabs(a[i] - b[i]));
    }
    return d;
  }

  static bool isSuspiciousDriverResponse(int ret, const std::string& message)
  {
    std::string lower = message;
    std::transform(lower.begin(), lower.end(), lower.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    if (ret != 1) {
      return true;
    }
    if (lower.find("err_" ) != std::string::npos ||
        lower.find("function_call_error") != std::string::npos ||
        lower.find("error") != std::string::npos) {
      return true;
    }
    return false;
  }

  double topicPosToMm(double v_topic) const
  {
    if (tool_pose_unit_ == "m") {
      return v_topic * 1000.0;
    }
    return v_topic;  // default mm
  }

  double toTopicDeltaFromMm(double mm) const
  {
    if (tool_pose_unit_ == "m") {
      return mm / 1000.0;
    }
    return mm;
  }

  bool waitForInitialData()
  {
    ros::Time start = ros::Time::now();
    ros::Rate rate(50.0);
    while (ros::ok()) {
      ros::spinOnce();
      if (has_pose_ && has_joint_state_) {
        return true;
      }
      if ((ros::Time::now() - start).toSec() > wait_pose_timeout_sec_) {
        return false;
      }
      rate.sleep();
    }
    return false;
  }

  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    latest_pose_ = *msg;
    if (!has_pose_) {
      initial_pose_ = latest_pose_;
      has_pose_ = true;
      ROS_INFO("[openloop_move_jaka4] Initial pose captured from topic.");
    }
  }

  void jointCallback(const sensor_msgs::JointState::ConstPtr& msg)
  {
    latest_joint_positions_ = msg->position;
    has_joint_state_ = true;
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber pose_sub_;
  ros::Subscriber joint_sub_;
  ros::ServiceClient linear_move_client_;

  geometry_msgs::PoseStamped latest_pose_;
  geometry_msgs::PoseStamped initial_pose_;
  std::vector<double> latest_joint_positions_;
  bool has_pose_{false};
  bool has_joint_state_{false};

  double total_distance_m_{0.10};
  double step_distance_m_{0.01};
  double total_distance_mm_{-1.0};
  double step_distance_mm_{-1.0};
  double small_test_step_mm_{50.0};

  double dwell_sec_{10.0};
  double command_settle_sec_{1.0};
  double wait_pose_timeout_sec_{8.0};
  double wait_motion_start_timeout_sec_{3.0};
  double wait_motion_done_timeout_sec_{20.0};
  double motion_joint_threshold_rad_{0.01};
  double motion_stable_duration_sec_{0.8};

  double active_total_mm_{100.0};
  double active_step_mm_{10.0};

  std::string arm_ns_;
  std::string tool_pose_topic_;
  std::string linear_move_service_;
  std::string joint_states_topic_;
  std::string tool_pose_unit_;
  std::string debug_motion_mode_;

  double mvvelo_{30.0};
  double mvacc_{30.0};
  double mvtime_{0.0};
  double mvradii_{0.0};
  int coord_mode_{0};
  int index_{0};
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "openloop_move_jaka4");

  OpenloopMoveJaka4 node;
  return node.run();
}
