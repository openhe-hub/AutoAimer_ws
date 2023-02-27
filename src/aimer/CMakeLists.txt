cmake_minimum_required(VERSION 3.0.2)
project(aimer)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} main.cpp)

add_subdirectory(auto_aim)
add_subdirectory(base)
add_subdirectory(param)




