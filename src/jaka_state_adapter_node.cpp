#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

class JakaStateAdapter
{
public:
  JakaStateAdapter()
  {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.param<std::string>("input_topic", input_topic_, "jaka_driver/joint_position");
    pnh.param<std::string>("output_topic", output_topic_, "joint_states");

    sub_ = nh.subscribe(input_topic_, 20, &JakaStateAdapter::jointPositionCallback, this);
    pub_ = nh.advertise<sensor_msgs::JointState>(output_topic_, 20);

    ROS_INFO("[jaka_state_adapter_node] started. input_topic=%s, output_topic=%s",
             input_topic_.c_str(), output_topic_.c_str());
  }

private:
  void jointPositionCallback(const sensor_msgs::JointState::ConstPtr& msg)
  {
    if (msg->position.size() < 6) {
      ROS_WARN_THROTTLE(1.0,
                        "[jaka_state_adapter_node] drop message: position.size()=%zu < 6",
                        msg->position.size());
      return;
    }

    sensor_msgs::JointState out;
    out.header.stamp = ros::Time::now();
    out.name = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
    out.position.assign(msg->position.begin(), msg->position.begin() + 6);

    if (msg->velocity.size() >= 6) {
      out.velocity.assign(msg->velocity.begin(), msg->velocity.begin() + 6);
    }
    if (msg->effort.size() >= 6) {
      out.effort.assign(msg->effort.begin(), msg->effort.begin() + 6);
    }

    pub_.publish(out);
  }

private:
  ros::Subscriber sub_;
  ros::Publisher pub_;
  std::string input_topic_;
  std::string output_topic_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "jaka_state_adapter_node");

  JakaStateAdapter node;
  ros::spin();
  return 0;
}
