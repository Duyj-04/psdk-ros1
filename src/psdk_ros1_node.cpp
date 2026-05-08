#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "psdk_ros1");
  ROS_INFO("psdk_ros1 node started");
  ros::spin();
  return 0;
}
