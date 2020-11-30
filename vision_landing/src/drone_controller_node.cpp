#include <vision_landing/DroneController.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "drone_controller");
  ros::NodeHandle nodeHandle("~");

  vision_landing::DroneController droneController(nodeHandle);
  
  ros::MultiThreadedSpinner spinner(4);
  spinner.spin(); 

  return 0;
}
