cmake_minimum_required(VERSION 3.16)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-std=c++14 -O2")

# opencv 
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# eigen 
include_directories("/usr/include/eigen3/")

# pcl 
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

# octomap 
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})

add_executable(pointcloud_mapping demo1_pointcloud_mapping.cpp)
target_link_libraries(pointcloud_mapping ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(octomap_mapping demo0_octomapping.cpp)
target_link_libraries(octomap_mapping ${OpenCV_LIBS} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_executable(pcd2octomap demo2_pcd2octomap.cpp)
target_link_libraries(pcd2octomap ${OpenCV_LIBS} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})
