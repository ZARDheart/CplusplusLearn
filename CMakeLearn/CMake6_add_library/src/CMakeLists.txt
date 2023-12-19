#这是生成共享库的CMakeLists.txt
#静态库：编译时将库源码复制到程序中，运行时不用库文件也能个运行，无需加载库，运行速度快，但占用内存大，且升级需要重新编译。
#动态库（共享库）：编译仅记录哪一个库的某一符号，运行的时候再调用，占用空间小，升级方便，无需重新编译，但程序运行需要加载库，耗费时间。这个demo就是一个动态库项目示例

#包含头文件：
include_directories(../include)

#为库链接三方库(示例，并没有在代码中用)
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

set(LIBHELLO_SRC hello.cpp hellowithpro.cpp)
#或 file(GLOB LIBHELLO_SRC "*.cpp")

# 生成共享库
#指令定义了：库文件的输出路径为 PROJECT/lib.正常编译后你就可以在 lib 目录得到一个 libhello.so，这就是我们期望的共享库
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
## set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
#PROJECT_BINARY_DIR指的编译发生的当前目录，如果是内部编译，就相当于 PROJECT_SOURCE_DIR 也就是工程代码所在目录，如果是外部编译，指的是外部编译所在目录，也就是本例中的 build目录。
add_library(hello SHARED ${LIBHELLO_SRC})
target_link_libraries(hello ${Pangolin_LIBRARIES})
# 为动态库设置版本
set_target_properties(hello PROPERTIES VERSION 1.2 SOVERSION 1)

# 生成静态库
# 上面有了同名的共享库，hello 作为一个 target 是不能重名的，因此
add_library(hello_static STATIC ${LIBHELLO_SRC}) 
target_link_libraries(hello_static ${Pangolin_LIBRARIES})
# 但是我们需要的是名字相同的静态库和动态库,增加一句：
set_target_properties(hello_static PROPERTIES OUTPUT_NAME "hello")
# 这样，我们就可以同时得到 libhello.so/libhello.a 两个库了

# 安装共享库和头文件
install(TARGETS hello hello_static
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib)
install(FILES ../include/hello.h DESTINATION include/hello)
# 注意，静态库要使用 ARCHIVE 关键字 通过:
# cmake -DCMAKE_INSTALL_PREFIX=~/Desktop/SLAM/LearnCMake/CMake6/install ..
# make
# make install
# 我们就可以将头文件和共享库安装到系统目录~/Desktop/SLAM/LearnCMake/CMake6/install/lib 和~/Desktop/SLAM/LearnCMake/CMake6/install/include/hello 中了
