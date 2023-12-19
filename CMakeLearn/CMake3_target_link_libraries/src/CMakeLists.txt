#包含自己的头文件目录：..代表上一级目录
include_directories(../include)

# 添加Eigen（只包含头文件的三方库）库文件
include_directories("/usr/include/eigen3")

# 添加Opencv库文件
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

file(GLOB SOURCES "../src/*.cpp")

add_executable(CMake3 ${SOURCES})

#链接库文件
target_link_libraries(CMake3 ${OpenCV_LIBS} )

set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
