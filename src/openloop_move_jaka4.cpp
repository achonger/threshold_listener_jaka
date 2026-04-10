#include <cmath>

#include <geometry_msgs/PoseStamped.h>
#include <jaka_msgs/Move.h>
#include <ros/ros.h>
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
    pnh_.param("dwell_sec", dwell_sec_, 10.0);
    pnh_.param("command_settle_sec", command_settle_sec_, 1.0);
    pnh_.param("wait_pose_timeout_sec", wait_pose_timeout_sec_, 8.0);
    pnh_.param<std::string>("arm_ns", arm_ns_, "jaka4");

    const std::string default_tool_pose_topic = "/" + arm_ns_ + "/jaka_driver/tool_position";
    const std::string default_linear_move_service = "/" + arm_ns_ + "/jaka_driver/linear_move";
    pnh_.param<std::string>("tool_pose_topic", tool_pose_topic_, default_tool_pose_topic);
    pnh_.param<std::string>("linear_move_service", linear_move_service_, default_linear_move_service);

    pnh_.param("mvvelo", mvvelo_, 30.0);
    pnh_.param("mvacc", mvacc_, 30.0);
    pnh_.param("mvtime", mvtime_, 0.0);
    pnh_.param("mvradii", mvradii_, 0.0);
    pnh_.param("coord_mode", coord_mode_, 0);
    pnh_.param("index", index_, 0);

    pose_sub_ = nh_.subscribe(tool_pose_topic_, 1, &OpenloopMoveJaka4::poseCallback, this);
    linear_move_client_ = nh_.serviceClient<jaka_msgs::Move>(linear_move_service_);
  }

  int run()
  {
    ROS_INFO("[openloop_move_jaka4] pose topic: %s", tool_pose_topic_.c_str());
    ROS_INFO("[openloop_move_jaka4] linear_move service: %s", linear_move_service_.c_str());

    if (!linear_move_client_.waitForExistence(ros::Duration(wait_pose_timeout_sec_))) {
      ROS_ERROR("[openloop_move_jaka4] Service unavailable within %.2f sec: %s",
                wait_pose_timeout_sec_, linear_move_service_.c_str());
      return 1;
    }

    if (!waitForInitialPose()) {
      ROS_ERROR("[openloop_move_jaka4] Failed to get initial pose within %.2f sec. Exit.", wait_pose_timeout_sec_);
      return 1;
    }

    const int steps = static_cast<int>(total_distance_m_ / step_distance_m_ + 1e-6);
    if (steps <= 0) {
      ROS_ERROR("[openloop_move_jaka4] Invalid step setup. total_distance_m=%.4f, step_distance_m=%.4f",
                total_distance_m_, step_distance_m_);
      return 1;
    }

    for (int i = 1; i <= steps && ros::ok(); ++i) {
      geometry_msgs::PoseStamped target = initial_pose_;
      target.pose.position.x = initial_pose_.pose.position.x - (step_distance_m_ * static_cast<double>(i));

      double roll = 0.0;
      double pitch = 0.0;
      double yaw = 0.0;
      tf2::Quaternion q(
        target.pose.orientation.x,
        target.pose.orientation.y,
        target.pose.orientation.z,
        target.pose.orientation.w);
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

      // jaka_driver linear_move service uses mm + rad convention.
      const double x_mm = target.pose.position.x * 1000.0;
      const double y_mm = target.pose.position.y * 1000.0;
      const double z_mm = target.pose.position.z * 1000.0;

      ROS_INFO("[openloop_move_jaka4] Step %d/%d target(m): x=%.6f y=%.6f z=%.6f rpy(rad)=%.6f %.6f %.6f",
               i, steps,
               target.pose.position.x, target.pose.position.y, target.pose.position.z,
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

      if (!linear_move_client_.call(srv)) {
        ROS_ERROR("[openloop_move_jaka4] Step %d service call failed.", i);
        return 1;
      }

      ROS_INFO("[openloop_move_jaka4] Step %d service returned: ret=%d, message=%s",
               i,
               srv.response.ret,
               srv.response.message.c_str());
      if (srv.response.ret != 1) {
        ROS_ERROR("[openloop_move_jaka4] Step %d rejected by driver: %s",
                  i,
                  srv.response.message.c_str());
        return 1;
      }

      ros::Duration(command_settle_sec_).sleep();
      ROS_INFO("[openloop_move_jaka4] Step %d dwell %.2f sec.", i, dwell_sec_);
      ros::Duration(dwell_sec_).sleep();
    }

    ROS_INFO("[openloop_move_jaka4] Completed %.3f m displacement.", total_distance_m_);
    return 0;
  }

private:
  bool waitForInitialPose()
  {
    ros::Time start = ros::Time::now();
    ros::Rate rate(50.0);
    while (ros::ok()) {
      ros::spinOnce();
      if (has_pose_) {
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
      ROS_INFO("[openloop_move_jaka4] Initial pose captured.");
    }
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber pose_sub_;
  ros::ServiceClient linear_move_client_;

  geometry_msgs::PoseStamped latest_pose_;
  geometry_msgs::PoseStamped initial_pose_;
  bool has_pose_{false};

  double total_distance_m_{0.10};
  double step_distance_m_{0.01};
  double dwell_sec_{10.0};
  double command_settle_sec_{1.0};
  double wait_pose_timeout_sec_{8.0};

  std::string arm_ns_;
  std::string tool_pose_topic_;
  std::string linear_move_service_;

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
