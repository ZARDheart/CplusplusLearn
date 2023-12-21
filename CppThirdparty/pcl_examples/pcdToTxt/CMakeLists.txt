cmake_minimum_required(VERSION 3.10)
project(ProjectName VERSION 1.0)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -std=c++14")

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcd2Txt_XYZI pcd2Txt_XYZI.cpp)
target_link_libraries(pcd2Txt_XYZI ${PCL_LIBRARIES})

add_executable(pcd2Txt_XYZRGB pcd2Txt_XYZRGB.cpp)
target_link_libraries(pcd2Txt_XYZRGB ${PCL_LIBRARIES})

add_executable(txt2Pcd_XYZRGB txt2Pcd_XYZRGB.cpp)
target_link_libraries(txt2Pcd_XYZRGB ${PCL_LIBRARIES})

add_executable(txt2Pcd_XYZI txt2Pcd_XYZI.cpp)
target_link_libraries(txt2Pcd_XYZI ${PCL_LIBRARIES})
