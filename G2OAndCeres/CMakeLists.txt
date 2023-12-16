cmake_minimum_required(VERSION 3.10)

project(G2OandCeres)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "-std=c++14 -O3")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Ceres
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

# g2o
# 找到FindG2O.cmake文件
LIST(APPEND CMAKE_MODULE_PATH /home/zard/Desktop/g2o/cmake_modules ) 
SET(G2O_ROOT /usr/local/include/g2o ) 
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

# Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/G2OAndCeres)

add_executable(gaussNewton gaussNewton.cpp)
target_link_libraries(gaussNewton ${OpenCV_LIBS} ${EIGEN3_LIBS})

add_executable(ceresCurveFitting ceresCurveFitting.cpp)
target_link_libraries(ceresCurveFitting ${OpenCV_LIBS} ${CERES_LIBRARIES})

add_executable(g2oCurveFitting g2oCurveFitting.cpp)
target_link_libraries(g2oCurveFitting ${OpenCV_LIBS} g2o_core g2o_stuff)
