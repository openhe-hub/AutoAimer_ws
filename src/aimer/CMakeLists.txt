cmake_minimum_required(VERSION 3.0.2)
project(aimer)

set(CMAKE_CXX_STANDARD 17)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  aimer_msgs

  UltraMultiThread
  base
  core_io
  common

  geometry_msgs
  sensor_msgs

  cv_bridge           # ros img conversion
  eigen_conversions   # ros eigen conversion
)

catkin_package(INCLUDE_DIRS include)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

find_package(Python 3.6 REQUIRED COMPONENTS Development Interpreter)
find_package(pybind11 REQUIRED)

include_directories(${Python_INCLUDE_DIRS})
include_directories(${pybind11_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} main.cpp)

target_link_libraries(${PROJECT_NAME} Python::Python pybind11::pybind11)

add_subdirectory(base)
add_subdirectory(param)
add_subdirectory(auto_aim)






