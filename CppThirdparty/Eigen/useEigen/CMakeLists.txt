set(CMAKE_CXX_FLAGS "-O3")

# 添加Eigen头文件
include_directories("/usr/include/eigen3")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin/Eigen)
add_executable(eigenMatrix eigenMatrix.cpp)
