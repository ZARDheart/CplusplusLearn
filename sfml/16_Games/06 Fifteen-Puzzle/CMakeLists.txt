cmake_minimum_required(VERSION 3.10)
project(ProjectName VERSION 1.0)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -std=c++14")

# Add executable
set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})
add_executable(Fifteen-Puzzle main.cpp)
target_link_libraries(Fifteen-Puzzle sfml-graphics sfml-window sfml-system)
