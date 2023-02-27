cmake_minimum_required(VERSION 3.15)

set(LIB_NAME heroaim_detector)

set(CMAKE_CXX_STANDARD 20)

find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

include_directories(${CERES_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

aux_source_directory(. src)

add_library(${LIB_NAME} OBJECT entry.cpp ${src})

# target_link_libraries(${LIB_NAME} ${TensorRT_LIBS} ${OpenCV_LIBS} fmt::fmt
# nvonnxparser MVSDK pthread cuda)
target_link_libraries(${LIB_NAME} ${OpenCV_LIBS} ${CERES_LIBRARIES} pthread)

target_link_libraries(${PROJECT_NAME} ${LIB_NAME})
