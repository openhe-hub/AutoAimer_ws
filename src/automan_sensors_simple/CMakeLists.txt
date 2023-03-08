cmake_minimum_required(VERSION 3.0.2)
project(automan_sensors_simple)

option(WITH_MINDVISION "enable mindvision driver" OFF)
option(WITH_HIKVISION "enable hikvision driver" ON)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV 4 REQUIRED)
find_package(fmt REQUIRED)
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  serial
  robot_msgs
  cv_bridge_4
)


if (WITH_MINDVISION)
    add_compile_definitions(WITH_MINDVISION)
    include_directories(include/MVSDK)
endif()
if (WITH_HIKVISION)
    add_compile_definitions(WITH_HIKVISION)
    
    set(HIK_ROOT /opt/MVS/lib/)
    find_library(HIK_CAMERA_CONTROL MvCameraControl
            HINTS ${HIK_ROOT}
            PATH_SUFFIXES 64)
    set(HIK_LIBS ${HIK_CAMERA_CONTROL})

    include_directories(include/HIKSDK)
endif()


catkin_package(
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${fmt_INCLUDE_DIRS}
  ./include
)

add_library(sensors_lib 
  src/sensors.cpp
  src/ExtImu.cpp
)
target_link_libraries(sensors_lib ${catkin_LIBRARIES})


add_executable(camera_imu src/camera_imu_node.cpp)
target_link_libraries(camera_imu ${OpenCV_LIBS} ${catkin_LIBRARIES} sensors_lib Threads::Threads)
if (WITH_MINDVISION)
    add_library(MINDVISION_SRC src/mindvision.cpp)
    target_link_libraries(camera_imu MINDVISION_SRC fmt::fmt MVSDK X11)
endif()
if (WITH_HIKVISION)
    add_library(HIKVISION_SRC src/hikvision.cpp)
    target_link_libraries(camera_imu HIKVISION_SRC fmt::fmt ${HIK_LIBS})
endif()