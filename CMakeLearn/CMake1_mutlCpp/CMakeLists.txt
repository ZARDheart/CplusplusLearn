#同文件夹下多个源码的情况：
#cmake要求的最小版本：
cmake_minimum_required(VERSION 3.0)

#C++11标准
add_compile_options(-std=c++11)

#项目名称及版本号：
project(CMake1 VERSION 1.0)

#目标执行文件-源码：
add_executable(CMake1 main.cpp testFunc.cpp )
