cmake_minimum_required(VERSION 3.16)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-std=c++14 -O2")

# opencv 
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# pcl 
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(surfel_mapping surfel_mapping.cpp)
target_link_libraries(surfel_mapping ${OpenCV_LIBS} ${PCL_LIBRARIES})
