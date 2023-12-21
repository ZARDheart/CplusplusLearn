#包含自己的头文件目录：..代表上一级目录
include_directories(../include)

# 添加Eigen（只包含头文件的三方库）库文件
include_directories("/usr/include/eigen3")

# 添加Opencv库文件
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#使用set或file设置多个源文件（依赖）：
#set(SOURCES src/main.cpp src/testFunc.cpp src/testFunc1.cpp)
file(GLOB SOURCES "../src/*.cpp")

MESSAGE(STATUS "This is SOURCE dir " ${CMake4_SOURCE_DIR})
#MESSAGE([SEND_ERROR | STATUS | FATAL_ERROR] "message to display"...)
#这个指令用于向终端输出用户定义的信息，包含了三种类型:
#SEND_ERROR，产生错误，生成过程被跳过。
#SATUS ，输出前缀为 — 的信息。FATAL_ERROR，立即终止所有 cmake 过程.
#我们在这里使用的是 STATUS 信息输出，演示了由 PROJECT 指令定义的两个隐式变量HELLO_BINARY_DIR 和 HELLO_SOURCE_DIR。

##目标执行文件-源码（使用变量SOURCE）：
#使用了${}来引用变量，这是 cmake 的变量应用方式，可生成多个子目标
#作为工程名的 CMake4 和生成的可执行文件 CMake4 是没有任何关系的
add_executable(CMake4 ${SOURCES})

#链接库文件
target_link_libraries(CMake4 ${OpenCV_LIBS} )

#重新定义:EXECUTABLE_OUTPUT_PATH 变量来指定最终的目标二进制的位置
#最终输出的elf文件会放到bin目录下
#EXECUTABLE_OUTPUT_PATH ：目标二进制可执行文件的存放位置
#PROJECT_SOURCE_DIR：工程的根目录
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#在哪里 ADD_EXECUTABLE，如果需要改变目标存放路径，就在哪里加入上述的定义。
