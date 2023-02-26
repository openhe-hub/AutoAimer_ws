cmake_minimum_required(VERSION 3.0.2)
project(detector)

set(LIB_NAME aimer_auto_aim_detector)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -flto")
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

aux_source_directory(./src SRC_FILES)

add_executable(${PROJECT_NAME} ${SRC_FILES})

target_link_directories(${PROJECT_NAME} PUBLIC ${OpenCV_LIBS} pthread)