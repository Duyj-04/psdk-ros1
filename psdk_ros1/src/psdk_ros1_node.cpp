#include <ros/ros.h>

#include <psdk_ros1/psdk_ros1_wrapper.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "psdk_ros1_node");
  ros::NodeHandle nh("psdk");
  ros::NodeHandle private_nh("~");

  psdk_ros1::PsdkRos1Wrapper wrapper(nh, private_nh);
  wrapper.start();

  ros::spin();
  wrapper.shutdown();
  return 0;
}
