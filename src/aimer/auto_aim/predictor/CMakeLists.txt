cmake_minimum_required(VERSION 3.15)

# set(lib_name aimer_auto_aim_predictor)

# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   std_msgs
#   aimer_msgs

#   UltraMultiThread
#   base
#   core_io
#   common

#   geometry_msgs
#   sensor_msgs

#   cv_bridge           # ros img conversion
#   eigen_conversions   # ros eigen conversion
# )
find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Python 3.6 REQUIRED COMPONENTS Development Interpreter)
find_package(pybind11 REQUIRED)
find_package(fmt 3 REQUIRED)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

# catkin_package()

# include_directories(
#   ${catkin_INCLUDE_DIRS}
# )

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${CERES_INCLUDE_DIRS})
include_directories(${Python_INCLUDE_DIRS})
include_directories(${pybind11_INCLUDE_DIRS})
# 搜集目录中所有源文件，存储到变量中
aux_source_directory(./aim aim_src)
aux_source_directory(./enemy enemy_src)
aux_source_directory(./motion motion_src)
aux_source_directory(./pnp pnp_src)
aux_source_directory(./enemy_predictor enemy_predictor_src)
aux_source_directory(. src)

# 编译变量的源文件并生成对象库 该库完全描述一个线程，不需要被链接
# add_library(${lib_name} OBJECT )

# 链接库
add_executable(predictor ${src})

target_link_libraries(predictor ${OpenCV_LIBS} ${CERES_LIBRARIES} pthread Python::Python pybind11::pybind11)
target_link_libraries(predictor
    ${catkin_LIBRARIES}
)
