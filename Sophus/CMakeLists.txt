#为使用 sophus，需要使用find_package命令找到它
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

# Eigen
include_directories("/usr/include/eigen3")
add_executable(useSophus useSophus.cpp)

#应遵循调用层次，从依赖到被依赖。所以在CMakeLists.txt中添加
target_link_libraries(useSophus ${Sophus_LIBRARIES} fmt)
