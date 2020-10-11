cmake_minimum_required(VERSION 3.0.2)
project(vision_landing)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  mavros
  mavros_msgs
  mavros_extras
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vision_landing
  CATKIN_DEPENDS roscpp rospy std_msgs mavros_msgs geometry_msgs sensor_msgs tf
)


include_directories(
include
${catkin_INCLUDE_DIRS}
)


add_library(vision_landing
  src/DroneController.cpp 
)
add_dependencies(vision_landing ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vision_landing ${catkin_LIBRARIES})


add_executable(drone_controller_node src/drone_controller_node.cpp)
target_link_libraries(drone_controller_node vision_landing ${catkin_LIBRARIES})