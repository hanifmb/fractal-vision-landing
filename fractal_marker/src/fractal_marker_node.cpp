#include <fractal_marker/FractalMarker.hpp>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ros_package_template");
  ros::NodeHandle nodeHandle("~");

  ROS_INFO("START");
  fractal_marker::FractalMarker fractalMarker(nodeHandle);

  ros::spin();
  return 0;
}
