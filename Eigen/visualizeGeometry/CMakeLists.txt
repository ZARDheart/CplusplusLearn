# 添加Eigen头文件
include_directories( "/usr/include/eigen3" )

# 添加Pangolin依赖
find_package(Pangolin)
include_directories( ${Pangolin_INCLUDE_DIRS})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Eigen/visualizeGeometry)

add_executable( visualizeGeometry visualizeGeometry.cpp )
target_link_libraries( visualizeGeometry ${Pangolin_LIBRARIES} )
