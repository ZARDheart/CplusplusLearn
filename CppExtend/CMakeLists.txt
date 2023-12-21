set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin/CppExtend)
add_executable(demo01_cmath demo01_cmath.cpp)
add_executable(demo02_yaml demo02_yaml.cpp)
target_link_libraries(demo02_yaml ${OpenCV_LIBS})