set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -std=c++14")

set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin/sfml)
add_executable(graph graph.cpp)
target_link_libraries(graph sfml-graphics sfml-window sfml-system)

add_executable(graph1 graph1.cpp)
target_link_libraries(graph1 sfml-graphics sfml-window sfml-system)
