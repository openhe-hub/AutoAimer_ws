cmake_minimum_required(VERSION 3.15)

# 构建子目录
# if("${auto_aim_network_engine}" STREQUAL "OPENCV")
add_subdirectory(detector_opencv)
# elseif("${auto_aim_network_engine}" STREQUAL "CUDA")
#   add_subdirectory(detector_cuda)
# elseif("${auto_aim_network_engine}" STREQUAL "OPENVINO")

# endif()

add_subdirectory(predictor)
