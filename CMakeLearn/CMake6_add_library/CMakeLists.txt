# 指令不区分大小写;
# 参数和变量：
# 严格区分大小写，名称中只有数字，字母，下划线和破折号;
# 用${}来引用变量;
# 参数之间空格分开;

# 标准cmake工程形式：
# 最外层build.sh用来执行编译
# 程序说明文档放入doc
# 一般会把库源文件放到src目录下，把库头文件放入到include文件下
# 编译生成的对象文件放入到build目录下（此文件夹下cmake）
# lib文件夹用来放生成共享库的库文件
# Examples放项目入口,其中又可细分源文件，头文件目录和配置文件目录,最终输出的elf文件放到bin目录下
# 以上内容使项目结构更加清晰，使用多个CMakeLists.txt，最外层的CMakeLists.txt用于掌控全局

# cmake要求的最小版本
cmake_minimum_required(VERSION 3.10)
# cmake 版本
MESSAGE("-- cmake version: " ${CMAKE_MAJOR_VERSION} "." ${CMAKE_MINOR_VERSION} "." ${CMAKE_PATCH_VERSION})


# PROJECT(projectname [CXX] [C] [Java])
# 用这个指令定义工程名称，并可指定工程支持的语言，支持的语言列表是可以忽略的，默认情况表示支持所有语言，可指定工程版本号
project(CMake5 VERSION 1.0)

# 编译模式Debug：详细调试信息，Release：无调试信息
SET(CMAKE_BUILD_TYPE Release)
MESSAGE("-- Build type: " ${CMAKE_BUILD_TYPE})

# C++11标准，也可以下面两种设置
# add_compile_options(-std=c++11)
# set(CMAKE_CXX_STANDARD 14)
# 设置编译选项变量与对应的值CMAKE_CXX_FLAGS,参数-Wall和-Werror显示所有的错误和警告,以及设置C++标准
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -std=c++14")
MESSAGE("-- cmake build mode: " ${CMAKE_CXX_FLAGS})

# add_subdirectory()命令可以向当前工程添加存放源文件的子目录
# 并可以指定中间二进制和目标二进制的存放位置
# 这里指定src目录下存放了源文件，当执行cmake时
# 就会进入src目录下去找src目录下建立的CMakeLists.txt
add_subdirectory(src) # 生成动态库
add_subdirectory(Examples/src) # 主函数链接动态库
