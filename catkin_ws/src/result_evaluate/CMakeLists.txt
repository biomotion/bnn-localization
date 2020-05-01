cmake_minimum_required(VERSION 2.8.3)
project(result_evaluate)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  sensor_msgs
  visualization_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES result_evaluate
#  CATKIN_DEPENDS geometry_msgs rospy sensor_msgs visualization_msgs
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
