cmake_minimum_required(VERSION 3.15)

# set(LIB_NAME aimer_auto_aim_detector)

set(CMAKE_CXX_STANDARD 20)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -flto")

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Python 3.6 REQUIRED COMPONENTS Development Interpreter)
find_package(pybind11 REQUIRED)
find_package(fmt 3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${Python_INCLUDE_DIRS})
include_directories(${pybind11_INCLUDE_DIRS})

aux_source_directory(. src)

# add_library(${LIB_NAME} OBJECT entry.cpp ${src})

add_executable(detector_opencv entry.cpp ${src})

# target_link_libraries(${LIB_NAME} )

target_link_libraries(detector_opencv ${OpenCV_LIBS} pthread Python::Python pybind11::pybind11 fmt::fmt)
target_link_libraries(detector_opencv
    ${catkin_LIBRARIES}
)
