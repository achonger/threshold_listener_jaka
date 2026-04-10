#include <ros/ros.h>
#include <std_msgs/Int32.h>

void thresholdCallback(const std_msgs::Int32::ConstPtr& msg)
{
    ROS_INFO("Received /threshold_detect: %d", msg->data);

    if (msg->data == 1) {
        ROS_INFO("threshold detected");
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "threshold_listener_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/threshold_detect", 10, thresholdCallback);

    ROS_INFO("threshold_listener_node started, waiting for /threshold_detect ...");
    ros::spin();

    return 0;
}
