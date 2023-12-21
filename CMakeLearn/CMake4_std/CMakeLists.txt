#工程形式：把源文件放到src目录下，把头文件放入到include文件下
#生成的对象文件放入到build目录下，最终输出的elf文件会放到bin目录下
#使用多个CMakeLists.txt，最外层的CMakeLists.txt用于掌控全局
cmake_minimum_required(VERSION 3.0)
#cmake要求的最小版本

#C++11标准
add_compile_options(-std=c++11)

#你可以用这个指令定义工程名称，并可指定工程支持的语言，支持的语言列表是可以忽略的，默认情况表示支持所有语言，及可指定工程版本号
project(CMake4 VERSION 1.0)

#add_subdirectory()命令可以向当前工程添加存放源文件的子目录
#并可以指定中间二进制和目标二进制的存放位置
#这里指定src目录下存放了源文件，当执行cmake时
#就会进入src目录下去找src目录下建立的CMakeLists.txt
add_subdirectory (src bin)
