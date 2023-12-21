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

add_executable(BoundingBox BoundingBox.cpp)
target_link_libraries (BoundingBox ${PCL_LIBRARIES} Boost::thread)
