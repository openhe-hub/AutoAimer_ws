cmake_minimum_required(VERSION 3.0.2)
project(UltraMultiThread LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

find_package(pybind11 REQUIRED)
find_package(Python 3.6 REQUIRED COMPONENTS Development Interpreter)

catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES UltraMultiThread
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)






