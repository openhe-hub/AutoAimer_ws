cmake_minimum_required(VERSION 3.15)

set(lib_name aimer_param)
set(entry_name aimer_param_entry)

find_package(OpenCV 4 REQUIRED)
find_package(fmt 3 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

# 同一个 cmake 下共享??
add_library(${lib_name} parameter.cpp)
add_library(${entry_name} OBJECT entry.cpp)

target_link_libraries(${lib_name} ${OpenCV_LIBS} pthread fmt::fmt)
target_link_libraries(${entry_name} ${lib_name} fmt::fmt)
# target_link_libraries(${PROJECT_NAME} ${entry_name})
