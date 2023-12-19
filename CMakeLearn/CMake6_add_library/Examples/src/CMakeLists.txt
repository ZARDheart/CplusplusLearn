# 利用自己编译的共享库

#包含入口的头文件目录和共享库的头文件目录：..代表上一级目录
include_directories(../include 
		../../include)

# 添加Eigen（只包含头文件的三方库）库文件
include_directories("/usr/include/eigen3")

# 添加Opencv库文件
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# 使用set或file设置多个源文件（依赖）：
# set(SOURCES src/main.cpp src/testFunc.cpp src/testFunc1.cpp)
# file(GLOB SOURCES "../src/*.cpp")
file(GLOB SOURCES "*.cpp")

MESSAGE(STATUS "This is SOURCE dir " ${CMake6_SOURCE_DIR})
# MESSAGE([SEND_ERROR | STATUS | FATAL_ERROR] "message to display"...)
# 这个指令用于向终端输出用户定义的信息，包含了三种类型:
# SEND_ERROR,产生错误，生成过程被跳过
# SATUS,输出前缀为 — 的信息
# FATAL_ERROR,立即终止所有 cmake 过程.
# 我们在这里使用的是 STATUS 信息输出，演示了由 PROJECT 指令定义的两个隐式变量HELLO_BINARY_DIR 和 HELLO_SOURCE_DIR。

# 重新定义:EXECUTABLE_OUTPUT_PATH 变量来指定最终的目标二进制的位置
# 最终输出的elf文件会放到bin目录下
# EXECUTABLE_OUTPUT_PATH ：目标二进制可执行文件的存放位置
# PROJECT_SOURCE_DIR：工程的根目录
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
# 这句与上面等价
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

# 目标执行文件-源码（使用变量SOURCE）：
# 使用了${}来引用变量，这是 cmake 的变量应用方式，可生成多个子目标
# 作为工程名的 CMake6 和生成的可执行文件 CMake6 是没有任何关系的
add_executable(CMake6 ${SOURCES})
# 链接库文件,包括三方库和自己编译的动态库
target_link_libraries(CMake6 
	${OpenCV_LIBS} 
        hello )
