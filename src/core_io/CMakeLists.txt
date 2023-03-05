cmake_minimum_required(VERSION 3.0.2)
project(core_io)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)








