cmake_minimum_required(VERSION 3.0.2)
project(aimer)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  aimer_msgs
  UltraMultiThread
  base
)

catkin_package(INCLUDE_DIRS include)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} main.cpp)

add_subdirectory(param)
add_subdirectory(auto_aim)
add_subdirectory(base)





