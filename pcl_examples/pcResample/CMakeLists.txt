cmake_minimum_required(VERSION 3.10)
project(pcResample)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)

# pcl 
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcResample pcResample.cpp)
target_link_libraries(pcResample ${PCL_LIBRARIES})
