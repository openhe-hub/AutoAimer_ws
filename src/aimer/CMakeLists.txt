cmake_minimum_required(VERSION 3.0.2)
project(aimer)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-fPIC)
# add_compile_definitions(-D_GLIBCXX_USE_CXX11_ABI=0)

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
find_package(fmt 3 REQUIRED)

include_directories(${Python_INCLUDE_DIRS})
include_directories(${pybind11_INCLUDE_DIRS})

add_library(${PROJECT_NAME} entry.cpp)

target_link_libraries(${PROJECT_NAME} Python::Python pybind11::pybind11 fmt::fmt)

add_subdirectory(base)
add_subdirectory(param)
add_subdirectory(auto_aim)






