set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -std=c++14")

# Add executable--------------------------------------
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin/LeetCode)
add_executable(1_Hash 1_Hash.cpp)
add_executable(2_ListNum2Add 2_ListNum2Add.cpp)
add_executable(3_Substring 3_Substring.cpp)

add_executable(26_removeDuplicates 26_removeDuplicates.cpp)
add_executable(202_isHappy 202_isHappy.cpp)