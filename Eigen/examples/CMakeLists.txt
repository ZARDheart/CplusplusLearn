include_directories("/usr/include/eigen3")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Eigen)
add_executable(coordinateTransform coordinateTransform.cpp)

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Eigen)
add_executable(plotTrajectory plotTrajectory.cpp)
target_link_libraries(plotTrajectory ${Pangolin_LIBRARIES})
