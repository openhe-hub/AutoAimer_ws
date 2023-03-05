cmake_minimum_required(VERSION 3.0.2)
project(common)


find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  UltraMultiThread
)

catkin_package(
 INCLUDE_DIRS include
)


include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/common.cpp
)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})



