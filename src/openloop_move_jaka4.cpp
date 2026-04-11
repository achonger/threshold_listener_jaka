#include <algorithm>
#include <cctype>
#include <cmath>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <jaka_msgs/Move.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Empty.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

class OpenloopMoveJaka4
{
public:
  OpenloopMoveJaka4()
    : pnh_("~")
  {
    pnh_.param<std::string>("arm_ns", arm_ns_, "jaka4");

    const std::string default_tool_pose_topic = "/" + arm_ns_ + "/jaka_driver/tool_position";
    const std::string default_joint_states_topic = "/" + arm_ns_ + "/joint_states";
    const std::string default_linear_move_service = "/" + arm_ns_ + "/jaka_driver/linear_move";
    const std::string default_stop_move_service = "/" + arm_ns_ + "/jaka_driver/stop_move";

    pnh_.param<std::string>("tool_pose_topic", tool_pose_topic_, default_tool_pose_topic);
    pnh_.param<std::string>("joint_states_topic", joint_states_topic_, default_joint_states_topic);
    pnh_.param<std::string>("linear_move_service", linear_move_service_, default_linear_move_service);
    pnh_.param<std::string>("stop_move_service", stop_move_service_, default_stop_move_service);
    pnh_.param<std::string>("threshold_topic", threshold_topic_, "/threshold_detect");

    pnh_.param<std::string>("tool_pose_unit", tool_pose_unit_, "mm");

    pnh_.param("step_distance_mm", step_distance_mm_, 10.0);
    pnh_.param("total_distance_mm", total_distance_mm_, 100.0);
    pnh_.param("dwell_sec", dwell_sec_, 10.0);

    pnh_.param("mvvelo", mvvelo_, 30.0);
    pnh_.param("mvacc", mvacc_, 30.0);
    pnh_.param("mvtime", mvtime_, 0.0);
    pnh_.param("mvradii", mvradii_, 0.0);
    pnh_.param("coord_mode", coord_mode_, 0);
    pnh_.param("index", index_, 0);

    pnh_.param("wait_pose_timeout_sec", wait_pose_timeout_sec_, 8.0);
    pnh_.param("wait_motion_start_timeout_sec", wait_motion_start_timeout_sec_, 3.0);
    pnh_.param("wait_motion_done_timeout_sec", wait_motion_done_timeout_sec_, 20.0);
    pnh_.param("motion_joint_threshold_rad", motion_joint_threshold_rad_, 0.01);
    pnh_.param("motion_stable_duration_sec", motion_stable_duration_sec_, 0.8);

    pose_sub_ = nh_.subscribe(tool_pose_topic_, 10, &OpenloopMoveJaka4::poseCallback, this);
    joint_sub_ = nh_.subscribe(joint_states_topic_, 20, &OpenloopMoveJaka4::jointCallback, this);
    threshold_sub_ = nh_.subscribe(threshold_topic_, 10, &OpenloopMoveJaka4::thresholdCallback, this);

    linear_move_client_ = nh_.serviceClient<jaka_msgs::Move>(linear_move_service_);
    stop_move_client_ = nh_.serviceClient<std_srvs::Empty>(stop_move_service_);
  }

  int run()
  {
    printConfig();

    if (!linear_move_client_.waitForExistence(ros::Duration(wait_pose_timeout_sec_))) {
      ROS_ERROR("[openloop_move_jaka4] linear_move service unavailable: %s", linear_move_service_.c_str());
      return 1;
    }

    if (!waitForInitialData()) {
      return 1;
    }

    printPose("initial", initial_pose_);

    const int max_steps = static_cast<int>(total_distance_mm_ / step_distance_mm_ + 1e-6);
    ROS_INFO("[openloop_move_jaka4] scan plan: max_steps=%d, step_distance_mm=%.3f, total_distance_mm=%.3f, dwell_sec=%.2f",
             max_steps, step_distance_mm_, total_distance_mm_, dwell_sec_);

    for (int step = 1; step <= max_steps && ros::ok(); ++step) {
      if (handleStopRequested("before sending command")) {
        return 0;
      }

      geometry_msgs::PoseStamped target = initial_pose_;
      const double cumulative_mm = step_distance_mm_ * static_cast<double>(step);
      target.pose.position.x = initial_pose_.pose.position.x - toTopicDeltaFromMm(cumulative_mm);

      const double current_x_mm = topicPosToMm(latest_pose_.pose.position.x);
      const double target_x_mm = topicPosToMm(target.pose.position.x);
      const double delta_x_mm = target_x_mm - current_x_mm;

      ROS_INFO("[openloop_move_jaka4] Step %d/%d, target_x_mm=%.3f, current_x_mm=%.3f, delta_x_mm=%.3f, cumulative_mm=%.3f",
               step, max_steps, target_x_mm, current_x_mm, delta_x_mm, cumulative_mm);

      if (!sendLinearTarget(target, step)) {
        return 1;
      }

      WaitResult wr = waitMotionStart();
      if (wr == WaitResult::STOPPED_BY_THRESHOLD) {
        return 0;
      }
      if (wr == WaitResult::TIMEOUT_OR_ERROR) {
        ROS_ERROR("[openloop_move_jaka4] Step %d failed: no effective motion start detected.", step);
        return 1;
      }

      wr = waitMotionSettle();
      if (wr == WaitResult::STOPPED_BY_THRESHOLD) {
        return 0;
      }
      if (wr == WaitResult::TIMEOUT_OR_ERROR) {
        ROS_ERROR("[openloop_move_jaka4] Step %d failed: motion did not settle in time.", step);
        return 1;
      }

      printPose("step_done", latest_pose_);

      if (cumulative_mm >= total_distance_mm_ - 1e-6) {
        ROS_INFO("[openloop_move_jaka4] reached max travel %.3f mm, final pose below:", total_distance_mm_);
        printPose("final", latest_pose_);
        return 0;
      }

      if (!dwellWithStopCheck(step)) {
        return 0;
      }
    }

    ROS_INFO("[openloop_move_jaka4] normal exit.");
    return 0;
  }

private:
  enum class WaitResult
  {
    OK,
    STOPPED_BY_THRESHOLD,
    TIMEOUT_OR_ERROR
  };

  bool sendLinearTarget(const geometry_msgs::PoseStamped& target, int step)
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

    ROS_INFO("[openloop_move_jaka4] Step %d target quaternion=[%.6f %.6f %.6f %.6f] rpy=[%.6f %.6f %.6f]",
             step,
             target.pose.orientation.x, target.pose.orientation.y,
             target.pose.orientation.z, target.pose.orientation.w,
             roll, pitch, yaw);

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
             step,
             srv.request.pose[0], srv.request.pose[1], srv.request.pose[2],
             srv.request.pose[3], srv.request.pose[4], srv.request.pose[5]);

    // Hard failure only when ROS service call itself fails.
    if (!linear_move_client_.call(srv)) {
      ROS_ERROR("[openloop_move_jaka4] Step %d linear_move service call failed.", step);
      return false;
    }

    ROS_INFO("[openloop_move_jaka4] Step %d linear_move response: ret=%d, message=%s",
             step, srv.response.ret, srv.response.message.c_str());

    if (isWarningLikeResponse(srv.response.ret, srv.response.message)) {
      ROS_WARN("[openloop_move_jaka4] Step %d response indicates warning; continue to observe real motion.", step);
    }

    return true;
  }

  WaitResult waitMotionStart()
  {
    const std::vector<double> baseline = latest_joint_positions_;
    ros::Time start = ros::Time::now();
    ros::Rate rate(100.0);

    while (ros::ok()) {
      ros::spinOnce();

      if (handleStopRequested("while waiting motion start")) {
        return WaitResult::STOPPED_BY_THRESHOLD;
      }

      if (has_joint_state_) {
        const double diff = maxJointDiff(baseline, latest_joint_positions_);
        if (diff > motion_joint_threshold_rad_) {
          ROS_INFO("[openloop_move_jaka4] joint motion detected, max_diff=%.6f rad", diff);
          return WaitResult::OK;
        }
      }

      if ((ros::Time::now() - start).toSec() > wait_motion_start_timeout_sec_) {
        return WaitResult::TIMEOUT_OR_ERROR;
      }
      rate.sleep();
    }

    return WaitResult::TIMEOUT_OR_ERROR;
  }

  WaitResult waitMotionSettle()
  {
    ros::Time start = ros::Time::now();
    ros::Time stable_since(0);
    std::vector<double> last = latest_joint_positions_;
    ros::Rate rate(100.0);

    while (ros::ok()) {
      ros::spinOnce();

      if (handleStopRequested("while waiting motion settle")) {
        return WaitResult::STOPPED_BY_THRESHOLD;
      }

      if (has_joint_state_) {
        const double diff = maxJointDiff(last, latest_joint_positions_);
        if (diff < motion_joint_threshold_rad_ * 0.5) {
          if (stable_since.isZero()) {
            stable_since = ros::Time::now();
          }
          if ((ros::Time::now() - stable_since).toSec() >= motion_stable_duration_sec_) {
            return WaitResult::OK;
          }
        } else {
          stable_since = ros::Time(0);
        }
        last = latest_joint_positions_;
      }

      if ((ros::Time::now() - start).toSec() > wait_motion_done_timeout_sec_) {
        return WaitResult::TIMEOUT_OR_ERROR;
      }
      rate.sleep();
    }

    return WaitResult::TIMEOUT_OR_ERROR;
  }

  bool dwellWithStopCheck(int step)
  {
    ROS_INFO("[openloop_move_jaka4] Step %d dwell %.2f sec", step, dwell_sec_);
    ros::Time start = ros::Time::now();
    ros::Rate rate(50.0);
    while (ros::ok()) {
      ros::spinOnce();
      if (handleStopRequested("during dwell")) {
        return false;
      }
      if ((ros::Time::now() - start).toSec() >= dwell_sec_) {
        return true;
      }
      rate.sleep();
    }
    return false;
  }

  bool handleStopRequested(const std::string& phase)
  {
    if (!stop_requested_) {
      return false;
    }

    ROS_WARN("[openloop_move_jaka4] stop requested (%s), calling stop_move service...", phase.c_str());
    std_srvs::Empty stop_srv;
    if (!stop_move_client_.call(stop_srv)) {
      ROS_WARN("[openloop_move_jaka4] stop_move service call failed: %s", stop_move_service_.c_str());
    } else {
      ROS_INFO("[openloop_move_jaka4] stop_move service call done.");
    }

    printPose("stopped_by_threshold", latest_pose_);
    return true;
  }

  bool waitForInitialData()
  {
    ros::Time start = ros::Time::now();
    ros::Rate rate(50.0);
    while (ros::ok()) {
      ros::spinOnce();

      if (handleStopRequested("while waiting initial data")) {
        return false;
      }

      if (has_pose_ && has_joint_state_) {
        return true;
      }
      if ((ros::Time::now() - start).toSec() > wait_pose_timeout_sec_) {
        if (has_pose_ && !has_joint_state_) {
          ROS_ERROR("[openloop_move_jaka4] initial tool pose received, but no joint_states on %s within %.2f sec",
                    joint_states_topic_.c_str(), wait_pose_timeout_sec_);
        } else if (!has_pose_ && has_joint_state_) {
          ROS_ERROR("[openloop_move_jaka4] joint_states received, but no tool pose on %s within %.2f sec",
                    tool_pose_topic_.c_str(), wait_pose_timeout_sec_);
        } else {
          ROS_ERROR("[openloop_move_jaka4] no initial pose/joint_states within %.2f sec", wait_pose_timeout_sec_);
        }
        return false;
      }
      rate.sleep();
    }
    return false;
  }

  void thresholdCallback(const std_msgs::Int32::ConstPtr& msg)
  {
    ROS_INFO("[openloop_move_jaka4] threshold_detect received: %d", msg->data);
    if (msg->data == 1) {
      stop_requested_ = true;
      ROS_WARN("[openloop_move_jaka4] threshold_detect == 1 -> stop requested");
    }
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

  void printConfig() const
  {
    ROS_INFO("[openloop_move_jaka4] arm_ns=%s", arm_ns_.c_str());
    ROS_INFO("[openloop_move_jaka4] tool_pose_topic=%s", tool_pose_topic_.c_str());
    ROS_INFO("[openloop_move_jaka4] joint_states_topic=%s", joint_states_topic_.c_str());
    ROS_INFO("[openloop_move_jaka4] linear_move_service=%s", linear_move_service_.c_str());
    ROS_INFO("[openloop_move_jaka4] stop_move_service=%s", stop_move_service_.c_str());
    ROS_INFO("[openloop_move_jaka4] threshold_topic=%s", threshold_topic_.c_str());
    ROS_INFO("[openloop_move_jaka4] tool_pose_unit=%s", tool_pose_unit_.c_str());
    ROS_INFO("[openloop_move_jaka4] step_distance_mm=%.3f total_distance_mm=%.3f dwell_sec=%.2f",
             step_distance_mm_, total_distance_mm_, dwell_sec_);
    ROS_INFO("[openloop_move_jaka4] move params: mvvelo=%.3f mvacc=%.3f mvtime=%.3f mvradii=%.3f coord_mode=%d index=%d",
             mvvelo_, mvacc_, mvtime_, mvradii_, coord_mode_, index_);
  }

  void printPose(const std::string& tag, const geometry_msgs::PoseStamped& pose) const
  {
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Quaternion q(
      pose.pose.orientation.x,
      pose.pose.orientation.y,
      pose.pose.orientation.z,
      pose.pose.orientation.w);
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    ROS_INFO("[openloop_move_jaka4] pose(%s): x=%.3f y=%.3f z=%.3f q=[%.6f %.6f %.6f %.6f] rpy=[%.6f %.6f %.6f]",
             tag.c_str(),
             topicPosToMm(pose.pose.position.x),
             topicPosToMm(pose.pose.position.y),
             topicPosToMm(pose.pose.position.z),
             pose.pose.orientation.x,
             pose.pose.orientation.y,
             pose.pose.orientation.z,
             pose.pose.orientation.w,
             roll, pitch, yaw);
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

  static bool isWarningLikeResponse(int ret, const std::string& message)
  {
    std::string lower = message;
    std::transform(lower.begin(), lower.end(), lower.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    if (ret != 1) {
      return true;
    }
    if (lower.find("err_") != std::string::npos ||
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

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber pose_sub_;
  ros::Subscriber joint_sub_;
  ros::Subscriber threshold_sub_;

  ros::ServiceClient linear_move_client_;
  ros::ServiceClient stop_move_client_;

  geometry_msgs::PoseStamped latest_pose_;
  geometry_msgs::PoseStamped initial_pose_;
  std::vector<double> latest_joint_positions_;

  bool has_pose_{false};
  bool has_joint_state_{false};
  bool stop_requested_{false};

  std::string arm_ns_;
  std::string tool_pose_topic_;
  std::string joint_states_topic_;
  std::string linear_move_service_;
  std::string stop_move_service_;
  std::string threshold_topic_;
  std::string tool_pose_unit_;

  double step_distance_mm_{10.0};
  double total_distance_mm_{100.0};
  double dwell_sec_{10.0};

  double total_distance_m_{0.10};
  double step_distance_m_{0.01};

  double mvvelo_{30.0};
  double mvacc_{30.0};
  double mvtime_{0.0};
  double mvradii_{0.0};
  int coord_mode_{0};
  int index_{0};

  double wait_pose_timeout_sec_{8.0};
  double wait_motion_start_timeout_sec_{3.0};
  double wait_motion_done_timeout_sec_{20.0};
  double motion_joint_threshold_rad_{0.01};
  double motion_stable_duration_sec_{0.8};
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "openloop_move_jaka4");

  OpenloopMoveJaka4 node;
  return node.run();
}
