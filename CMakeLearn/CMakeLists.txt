cmake_minimum_required(VERSION 3.10)
project(ProjectName VERSION 1.0)

# 这是一个示例CMakeLists.txt文件

# SET(CMAKE_BUILD_TYPE Release/Debug)
# MESSAGE("-- Build type: " ${CMAKE_BUILD_TYPE})

# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Werror -std=c++14")
# MESSAGE("-- cmake build mode: " ${CMAKE_C_FLAGS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -std=c++14")
MESSAGE("-- cmake build mode: " ${CMAKE_CXX_FLAGS})

# MESSAGE([SEND_ERROR | STATUS | FATAL_ERROR] "message to display"...)

# Include directories---------------------------------
# include_directories({PROJECT_SOURCE_DIR}/include)


# Find package----------------------------------------
# find_package(thirdparty REQUIRED)
# include_directories(${thirdparty_INCLUDE_DIRS})


# Find sources----------------------------------------
# file(GLOB LIBHELLO_SRC "{PROJECT_SOURCE_DIR}/src/*.cpp")
# file(GLOB SOURCES "{PROJECT_SOURCE_DIR}/Examples/src/*.cpp")


# Add_library-----------------------------------------
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
# add_library(hello SHARED ${LIBHELLO_SRC})
# target_link_libraries(hello ${thirdparty_LIBRARIES})

# add_library(hello_static STATIC ${LIBHELLO_SRC}) 
# target_link_libraries(hello_static ${Pangolin_LIBRARIES})
# set_target_properties(hello_static PROPERTIES OUTPUT_NAME "hello")


# install library-------------------------------------
# install(TARGETS hello hello_static
#	LIBRARY DESTINATION lib
#	ARCHIVE DESTINATION lib)
# install(FILES ../include/hello.h DESTINATION include/hello)


# Add executable--------------------------------------
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
# add_executable(CMake ${SOURCES})
# target_link_libraries(CMake ${hello_LIBS})
# target_link_libraries(CMake ${CMake5_SOURCE_DIR}/install/lib/libhello.so)


# Add subdirectory------------------------------------
# add_subdirectory(directory)
