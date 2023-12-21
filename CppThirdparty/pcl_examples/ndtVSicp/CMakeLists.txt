cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(NDTVSICP VERSION 1.0)

SET(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -std=c++14")
MESSAGE("-- cmake build mode: " ${CMAKE_CXX_FLAGS})

find_package(PCL 1.10 REQUIRED)
find_package(Boost REQUIRED COMPONENTS timer thread serialization chrono)
include_directories(
    ${PCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(normal_distributions_transform normal_distributions_transform.cpp)
target_link_libraries (normal_distributions_transform ${PCL_LIBRARIES})

add_executable(Iterative_Closest_Point_p2p Iterative_Closest_Point_p2p.cpp)
target_link_libraries (Iterative_Closest_Point_p2p ${PCL_LIBRARIES})

add_executable(icp_and_ndt icp_and_ndt.cpp)
target_link_libraries (icp_and_ndt ${PCL_LIBRARIES})

add_executable(Direct_synthesis Direct_synthesis.cpp)
target_link_libraries (Direct_synthesis ${PCL_LIBRARIES})

add_executable(clouds_viewer clouds_viewer.cpp)
target_link_libraries (clouds_viewer ${PCL_LIBRARIES})

add_executable(One_data_transform_NDT One_data_transform_NDT.cpp)
target_link_libraries (One_data_transform_NDT ${PCL_LIBRARIES})

add_executable(One_data_transform_ICP One_data_transform_ICP.cpp)
target_link_libraries (One_data_transform_ICP ${PCL_LIBRARIES})
