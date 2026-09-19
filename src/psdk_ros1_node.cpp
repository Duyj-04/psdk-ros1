#include <ros/ros.h>

#include <psdk_ros1/psdk_ros1_wrapper.hpp>

// Thin ROS entry point.  The wrapper owns the PSDK lifecycle and all service,
// topic, timer, and callback wiring; this file only supplies the ROS spin loop.
int main(int argc, char** argv)
{
  ros::init(argc, argv, "psdk_ros1_node");
  ros::NodeHandle nh("psdk");
  ros::NodeHandle private_nh("~");

  psdk_ros1::PsdkRos1Wrapper wrapper(nh, private_nh);
  // start() publishes initialization failures through the wrapper's state
  // topics, so the process can still remain observable when hardware is absent.
  wrapper.start();

  ros::spin();
  // Keep shutdown explicit so module deinitialization happens before process
  // exit rather than relying only on the C++ destructor order.
  wrapper.shutdown();
  return 0;
}
