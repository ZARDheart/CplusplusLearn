#标准工程形式：一般会把源文件放到src目录下，把头文件放入到include文件下
#生成的对象文件放入到build目录下，最终输出的elf文件会放到bin目录下，结构更加清晰。
cmake_minimum_required(VERSION 3.0)

project(CMake2 VERSION 1.0)

#设置编译选项变量与对应的值CMAKE_CXX_FLAGS,参数-Wall和-Werror显示所有的错误和警告,以及设置C++标准
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -std=c++14")

#包含自己的头文件目录：
include_directories(include)

#使用set或file设置源文件（依赖）：
#set(SOURCES src/main.cpp src/testFunc.cpp)
file(GLOB SOURCES "src/*.cpp")

#目标执行文件-源码（使用变量SOURCE）：
add_executable(CMake2 ${SOURCES})

#最终输出的elf文件会放到bin目录下
#EXECUTABLE_OUTPUT_PATH ：目标二进制可执行文件的存放位置
#PROJECT_SOURCE_DIR：工程的根目录
set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
