cmake_minimum_required(VERSION 2.8.3)
project(pick_objects)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  move_base_msgs
  actionlib
  rosapp
)

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES simple_arm
  CATKIN_DEPENDS actionlib move_base_msgs roscpp
  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(pick_objects src/pick_objects.cpp)
target_link_libraries(pick_objects ${catkin_LIBRARIES})
