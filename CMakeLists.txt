cmake_minimum_required(VERSION 3.10)
project(cpluslearn)

set(CMAKE_BUILD_TYPE Release)
# 使用线程需添加c++ 11标准支持，并且thread库不是Linux默认库，需加 -pthread
set(CMAKE_CXX_FLAGS "${CAMKE_CXX_FLAGS} -std=c++11 -pthread")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_subdirectory(CppBase)
add_subdirectory(CppCore)
add_subdirectory(CppSTL)
add_subdirectory(CppExtend)

# add_subdirectory(CppThirdparty/G2OAndCeres)
add_subdirectory(CppThirdparty/Eigen)
add_subdirectory(CppThirdparty/gtsam_examples)
add_subdirectory(CppThirdparty/opencv)
add_subdirectory(CppThirdparty/sfml)
# add_subdirectory(CppThirdparty/Sophus)
