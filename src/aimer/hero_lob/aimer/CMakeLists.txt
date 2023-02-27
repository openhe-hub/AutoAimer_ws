cmake_minimum_required(VERSION 3.15)

set(LIB_NAME heroaim_aimer)

find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${CERES_INCLUDE_DIRS})

# 添加头文件搜索路径
include_directories(.)
# 搜集目录中所有源文件，存储到变量中
aux_source_directory(. src)

# 编译变量的源文件并生成对象库，不链接
add_library(${LIB_NAME} OBJECT ${src})

# 链接库
target_link_libraries(${LIB_NAME} ${OpenCV_LIBS} ${CERES_LIBRARIES})
target_link_libraries(${lib_name} aimer_base)
# target_link_libraries(${lib_name} aimer_param)

target_link_libraries(${PROJECT_NAME} ${LIB_NAME})
