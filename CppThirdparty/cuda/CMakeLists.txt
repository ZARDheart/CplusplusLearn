# cmake_minimum_required(VERSION 3.8 FATAL_ERROR)

# project(cmake_and_cuda)
# project(ndt_gpu CUDA CXX) # 这个写法也是可以的

set(CMAKE_CUDA_ARCHITECTURES 86)
#set(CMAKE_CUDA_COMPILER /usr/local/cuda-11.6/bin/nvcc)
#set(CUDACXX /usr/local/cuda-11.6/bin/nvcc)
# 激活CUDA语言支持，project使用第1个写法时要进行注释
enable_language(CUDA) 
# 设置语言C++11特性
set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_CUDA_STANDARD 11)

find_package(CUDA REQUIRED)
include_directories(
    include
    ${CUDA_INCLUDE_DIRS}
    )

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin/cuda)
add_executable(cuda_cpp main.cpp)

add_subdirectory(kernels)
# set_property(TARGET cuda_cpp 
#              PROPERTY CUDA_SEPARABLE_COMPILATION ON)
target_link_libraries(cuda_cpp kernels ${CUDA_LIBRARIES})
