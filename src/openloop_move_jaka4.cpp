#include <cmath>
#include <string>
#include <thread>

#include <geometry_msgs/PoseStamped.h>
#include <jaka_msgs/Move.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Empty.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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

    pnh_.param("line_distance_mm", line_distance_mm_, 80.0);
    pnh_.param("direction_x", direction_x_, -1.0);
    pnh_.param("direction_y", direction_y_, 0.0);
    pnh_.param("direction_z", direction_z_, 0.0);
    pnh_.param<std::string>("direction_frame", direction_frame_, "base");
    pnh_.param("line_speed_mm_s", line_speed_mm_s_, 5.0);
    pnh_.param("line_acc_mm_s2", line_acc_mm_s2_, 20.0);

    pnh_.param("wait_pose_timeout_sec", wait_pose_timeout_sec_, 8.0);
    pnh_.param("threshold_status_log_interval_sec", threshold_status_log_interval_sec_, 2.0);
    pnh_.param("motion_time_safety_margin_sec", motion_time_safety_margin_sec_, 5.0);

    pnh_.param("mvtime", mvtime_, 0.0);
    pnh_.param("mvradii", mvradii_, 0.0);
    pnh_.param("coord_mode", coord_mode_, 0);
    pnh_.param("index", index_, 0);

    pose_sub_ = nh_.subscribe(tool_pose_topic_, 10, &OpenloopMoveJaka4::poseCallback, this);
    joint_sub_ = nh_.subscribe(joint_states_topic_, 20, &OpenloopMoveJaka4::jointCallback, this);
    threshold_sub_ = nh_.subscribe(threshold_topic_, 10, &OpenloopMoveJaka4::thresholdCallback, this);

    linear_move_client_ = nh_.serviceClient<jaka_msgs::Move>(linear_move_service_);
    stop_move_client_ = nh_.serviceClient<std_srvs::Empty>(stop_move_service_);
  }

  int run()
  {
    ros::AsyncSpinner spinner(2);
    spinner.start();

    printConfig();
    logThresholdMonitorStatus("startup", true);

    if (!linear_move_client_.waitForExistence(ros::Duration(wait_pose_timeout_sec_))) {
      ROS_ERROR("[openloop_move_jaka4] linear_move service unavailable: %s", linear_move_service_.c_str());
      return 1;
    }

    if (!waitForInitialData()) {
      return 1;
    }

    if (handleStopRequested("before planning linear move")) {
      return 0;
    }

    printPose("initial", initial_pose_);

    double ux = direction_x_;
    double uy = direction_y_;
    double uz = direction_z_;
    if (!computeDirectionUnit(ux, uy, uz)) {
      return 1;
    }

    geometry_msgs::PoseStamped target = initial_pose_;
    target.pose.position.x = initial_pose_.pose.position.x + toTopicDeltaFromMm(ux * line_distance_mm_);
    target.pose.position.y = initial_pose_.pose.position.y + toTopicDeltaFromMm(uy * line_distance_mm_);
    target.pose.position.z = initial_pose_.pose.position.z + toTopicDeltaFromMm(uz * line_distance_mm_);

    printPose("target", target);
    const double expected_motion_time_sec = line_distance_mm_ / std::max(1e-6, line_speed_mm_s_);
    ROS_INFO("[openloop_move_jaka4] estimated motion time: %.3f sec (safety_margin=%.3f sec, total_watch=%.3f sec)",
             expected_motion_time_sec, motion_time_safety_margin_sec_, expected_motion_time_sec + motion_time_safety_margin_sec_);

    if (!sendLinearTarget(target)) {
      return 1;
    }

    ROS_INFO("[openloop_move_jaka4] Linear command sent once. Please verify actual robot motion visually.");

    const ros::Time start_watch = ros::Time::now();
    const double total_watch_sec = expected_motion_time_sec + motion_time_safety_margin_sec_;
    ros::Rate rate(50.0);
    while (ros::ok()) {
      logThresholdMonitorStatus("monitoring linear motion", false);
      if (handleStopRequested("during linear motion monitoring")) {
        return 0;
      }
      if ((ros::Time::now() - start_watch).toSec() >= total_watch_sec) {
        break;
      }
      rate.sleep();
    }

    printPose("final", latest_pose_);
    ROS_INFO("[openloop_move_jaka4] motion monitor completed without threshold stop; normal exit.");
    return 0;
  }

private:
  bool computeDirectionUnit(double& ux, double& uy, double& uz) const
  {
    const double norm = std::sqrt(ux * ux + uy * uy + uz * uz);
    ROS_INFO("[openloop_move_jaka4] direction input raw=[%.6f, %.6f, %.6f], frame=%s",
             ux, uy, uz, direction_frame_.c_str());
    if (norm < 1e-9) {
      ROS_ERROR("[openloop_move_jaka4] direction norm too small: %.12f", norm);
      return false;
    }
    ux /= norm;
    uy /= norm;
    uz /= norm;

    if (direction_frame_ == "tool") {
      tf2::Quaternion q(
        initial_pose_.pose.orientation.x,
        initial_pose_.pose.orientation.y,
        initial_pose_.pose.orientation.z,
        initial_pose_.pose.orientation.w);
      tf2::Matrix3x3 rot(q);
      tf2::Vector3 tool_dir(ux, uy, uz);
      tf2::Vector3 base_dir = rot * tool_dir;
      ux = base_dir.x();
      uy = base_dir.y();
      uz = base_dir.z();
      ROS_INFO("[openloop_move_jaka4] direction_frame=tool converted to base direction=[%.6f, %.6f, %.6f]",
               ux, uy, uz);
      return true;
    }

    if (direction_frame_ != "base") {
      ROS_WARN("[openloop_move_jaka4] unsupported direction_frame=%s, fallback to base", direction_frame_.c_str());
    }

    ROS_INFO("[openloop_move_jaka4] normalized base direction=[%.6f, %.6f, %.6f]", ux, uy, uz);
    return true;
  }

  bool sendLinearTarget(const geometry_msgs::PoseStamped& target)
  {
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    tf2::Quaternion q(
      target.pose.orientation.x,
      target.pose.orientation.y,
      target.pose.orientation.z,
      target.pose.orientation.w);
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    const double x_mm = topicPosToMm(target.pose.position.x);
    const double y_mm = topicPosToMm(target.pose.position.y);
    const double z_mm = topicPosToMm(target.pose.position.z);

    jaka_msgs::Move srv;
    srv.request.pose = {
      static_cast<float>(x_mm),
      static_cast<float>(y_mm),
      static_cast<float>(z_mm),
      static_cast<float>(roll),
      static_cast<float>(pitch),
      static_cast<float>(yaw)
    };
    srv.request.has_ref = false;
    srv.request.ref_joint.clear();
    srv.request.mvvelo = line_speed_mm_s_;
    srv.request.mvacc = line_acc_mm_s2_;
    srv.request.mvtime = mvtime_;
    srv.request.mvradii = mvradii_;
    srv.request.coord_mode = coord_mode_;
    srv.request.index = index_;

    ROS_INFO("[openloop_move_jaka4] linear_move request.pose=[%.3f, %.3f, %.3f, %.6f, %.6f, %.6f]",
             srv.request.pose[0], srv.request.pose[1], srv.request.pose[2],
             srv.request.pose[3], srv.request.pose[4], srv.request.pose[5]);

    if (!linear_move_client_.call(srv)) {
      ROS_ERROR("[openloop_move_jaka4] linear_move service call failed.");
      return false;
    }

    ROS_INFO("[openloop_move_jaka4] linear_move response: ret=%d, message=%s (ignored for flow control)",
             srv.response.ret, srv.response.message.c_str());
    return true;
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
      logThresholdMonitorStatus("waiting initial data", false);
      if (handleStopRequested("while waiting initial data")) {
        return false;
      }
      if (has_pose_ && has_joint_state_) {
        return true;
      }
      if ((ros::Time::now() - start).toSec() > wait_pose_timeout_sec_) {
        ROS_ERROR("[openloop_move_jaka4] waiting initial pose/joint_state timeout %.2f sec", wait_pose_timeout_sec_);
        return false;
      }
      rate.sleep();
    }
    return false;
  }

  void thresholdCallback(const std_msgs::Int32::ConstPtr& msg)
  {
    threshold_msg_received_ = true;
    ++threshold_msg_count_;
    last_threshold_value_ = msg->data;
    last_threshold_stamp_ = ros::Time::now();
    ROS_INFO("[openloop_move_jaka4] Received %s: %d (count=%zu, stamp=%.3f)",
             threshold_topic_.c_str(), msg->data, threshold_msg_count_, last_threshold_stamp_.toSec());
    if (msg->data == 1) {
      stop_requested_ = true;
      threshold_triggered_ = true;
      ROS_WARN("[openloop_move_jaka4] Received %s: 1 -> stop requested", threshold_topic_.c_str());
    } else {
      ROS_INFO("[openloop_move_jaka4] Received %s: %d -> no stop requested", threshold_topic_.c_str(), msg->data);
    }
  }

  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    latest_pose_ = *msg;
    if (!has_pose_) {
      initial_pose_ = latest_pose_;
      has_pose_ = true;
      ROS_INFO("[openloop_move_jaka4] Initial pose captured.");
    }
  }

  void jointCallback(const sensor_msgs::JointState::ConstPtr& msg)
  {
    latest_joint_positions_ = msg->position;
    has_joint_state_ = true;
  }

  void logThresholdMonitorStatus(const std::string& phase, bool force)
  {
    const ros::Time now = ros::Time::now();
    if (!force && !last_threshold_status_log_time_.isZero() &&
        (now - last_threshold_status_log_time_).toSec() < threshold_status_log_interval_sec_) {
      return;
    }
    last_threshold_status_log_time_ = now;
    const uint32_t pub_count = threshold_sub_.getNumPublishers();
    if (!threshold_msg_received_) {
      if (pub_count == 0) {
        ROS_WARN("[openloop_move_jaka4] threshold monitor (%s): subscribed to %s, no publisher on topic yet",
                 phase.c_str(), threshold_topic_.c_str());
      } else {
        ROS_INFO("[openloop_move_jaka4] threshold monitor (%s): waiting for %s ... no message received yet (publishers=%u)",
                 phase.c_str(), threshold_topic_.c_str(), pub_count);
      }
      return;
    }
    const double age_sec = std::max(0.0, (now - last_threshold_stamp_).toSec());
    ROS_INFO("[openloop_move_jaka4] threshold monitor (%s): last_value=%d, last_msg_age=%.2f s, msg_count=%zu, publishers=%u, triggered=%s",
             phase.c_str(), last_threshold_value_, age_sec, threshold_msg_count_, pub_count, threshold_triggered_ ? "true" : "false");
  }

  void printConfig() const
  {
    ROS_INFO("[openloop_move_jaka4] arm_ns=%s", arm_ns_.c_str());
    ROS_INFO("[openloop_move_jaka4] tool_pose_topic=%s", tool_pose_topic_.c_str());
    ROS_INFO("[openloop_move_jaka4] joint_states_topic=%s", joint_states_topic_.c_str());
    ROS_INFO("[openloop_move_jaka4] linear_move_service=%s", linear_move_service_.c_str());
    ROS_INFO("[openloop_move_jaka4] stop_move_service=%s", stop_move_service_.c_str());
    ROS_INFO("[openloop_move_jaka4] threshold_topic=%s", threshold_topic_.c_str());
    ROS_INFO("[openloop_move_jaka4] line_distance_mm=%.3f line_speed_mm_s=%.3f line_acc_mm_s2=%.3f",
             line_distance_mm_, line_speed_mm_s_, line_acc_mm_s2_);
    ROS_INFO("[openloop_move_jaka4] direction=[%.6f, %.6f, %.6f], direction_frame=%s",
             direction_x_, direction_y_, direction_z_, direction_frame_.c_str());
  }

  void printPose(const std::string& tag, const geometry_msgs::PoseStamped& pose) const
  {
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
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

  double topicPosToMm(double v_topic) const
  {
    return (tool_pose_unit_ == "m") ? (v_topic * 1000.0) : v_topic;
  }

  double toTopicDeltaFromMm(double mm) const
  {
    return (tool_pose_unit_ == "m") ? (mm / 1000.0) : mm;
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
  bool threshold_msg_received_{false};
  bool threshold_triggered_{false};
  int last_threshold_value_{0};
  size_t threshold_msg_count_{0};
  ros::Time last_threshold_stamp_;
  ros::Time last_threshold_status_log_time_;

  std::string arm_ns_;
  std::string tool_pose_topic_;
  std::string joint_states_topic_;
  std::string linear_move_service_;
  std::string stop_move_service_;
  std::string threshold_topic_;
  std::string tool_pose_unit_;
  std::string direction_frame_;

  double line_distance_mm_{80.0};
  double direction_x_{-1.0};
  double direction_y_{0.0};
  double direction_z_{0.0};
  double line_speed_mm_s_{5.0};
  double line_acc_mm_s2_{20.0};
  double wait_pose_timeout_sec_{8.0};
  double threshold_status_log_interval_sec_{2.0};
  double motion_time_safety_margin_sec_{5.0};
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
