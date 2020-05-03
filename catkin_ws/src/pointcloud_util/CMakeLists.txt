cmake_minimum_required(VERSION 2.8.3)
project(pointcloud_util)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
)
find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pointcloud_util
#  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
add_executable(read_file src/read_file.cpp)
target_link_libraries(read_file ${catkin_LIBRARIES})

add_executable(downsampling src/downsampling.cpp)
target_link_libraries(downsampling ${catkin_LIBRARIES})


add_executable(read_gt src/read_gdtruth.cpp)
target_link_libraries(read_gt ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(outlier_removal src/outlier_removal.cpp)
target_link_libraries(outlier_removal ${catkin_LIBRARIES})

add_executable(ground_filter src/ground_filter.cpp)
target_link_libraries(ground_filter ${catkin_LIBRARIES})
