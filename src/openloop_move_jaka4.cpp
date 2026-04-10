#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

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

    const std::string default_tool_pose_topic = "/" + arm_ns_ + "/tool_position";
    const std::string default_linear_move_topic = "/" + arm_ns_ + "/linear_move";
    pnh_.param<std::string>("tool_pose_topic", tool_pose_topic_, default_tool_pose_topic);
    pnh_.param<std::string>("linear_move_topic", linear_move_topic_, default_linear_move_topic);

    pose_sub_ = nh_.subscribe(tool_pose_topic_, 1, &OpenloopMoveJaka4::poseCallback, this);
    linear_move_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(linear_move_topic_, 1, false);
  }

  int run()
  {
    ROS_INFO("[openloop_move_jaka4] Node started. Waiting for initial pose from [%s] ...", tool_pose_topic_.c_str());

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

    ROS_INFO("[openloop_move_jaka4] Initial pose: x=%.6f, y=%.6f, z=%.6f, q=[%.6f, %.6f, %.6f, %.6f]",
             initial_pose_.pose.position.x,
             initial_pose_.pose.position.y,
             initial_pose_.pose.position.z,
             initial_pose_.pose.orientation.x,
             initial_pose_.pose.orientation.y,
             initial_pose_.pose.orientation.z,
             initial_pose_.pose.orientation.w);

    ROS_INFO("[openloop_move_jaka4] Plan: %d steps, each %.3f m, total %.3f m along negative X.",
             steps,
             step_distance_m_,
             total_distance_m_);

    for (int i = 1; i <= steps && ros::ok(); ++i) {
      geometry_msgs::PoseStamped target = initial_pose_;
      target.header.stamp = ros::Time::now();
      target.pose.position.x = initial_pose_.pose.position.x - (step_distance_m_ * static_cast<double>(i));

      ROS_INFO("[openloop_move_jaka4] Step %d/%d, target x=%.6f, y=%.6f, z=%.6f",
               i,
               steps,
               target.pose.position.x,
               target.pose.position.y,
               target.pose.position.z);

      linear_move_pub_.publish(target);
      ROS_INFO("[openloop_move_jaka4] Step %d command published to [%s].", i, linear_move_topic_.c_str());

      ros::Duration(command_settle_sec_).sleep();
      ROS_INFO("[openloop_move_jaka4] Step %d settle wait %.2f sec done.", i, command_settle_sec_);

      ROS_INFO("[openloop_move_jaka4] Step %d dwell for %.2f sec ...", i, dwell_sec_);
      ros::Duration(dwell_sec_).sleep();
    }

    ROS_INFO("[openloop_move_jaka4] Completed %.3f m total displacement on negative X. Exit.", total_distance_m_);
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
      ROS_INFO("[openloop_move_jaka4] Initial pose captured from [%s].", tool_pose_topic_.c_str());
    }
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber pose_sub_;
  ros::Publisher linear_move_pub_;

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
  std::string linear_move_topic_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "openloop_move_jaka4");

  OpenloopMoveJaka4 node;
  return node.run();
}
