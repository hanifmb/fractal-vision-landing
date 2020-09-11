cmake_minimum_required(VERSION 3.0.2)
project(fractal_marker)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 11) # C++11...
set(CMAKE_CXX_STANDARD_REQUIRED ON) #...is required...
set(CMAKE_CXX_EXTENSIONS ON) #...with compiler extensions like gnu++11

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Boost REQUIRED COMPONENTS system)
find_package(aruco REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf 
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs tf
)

include_directories(

include

${catkin_INCLUDE_DIRS}

)

add_executable(fractal_marker_node src/fractal_marker_node.cpp src/FractalMarker.cpp)
target_link_libraries(fractal_marker_node aruco opencv_calib3d)
target_link_libraries(fractal_marker_node ${catkin_LIBRARIES})
