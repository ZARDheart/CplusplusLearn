set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin/CppCore)
add_executable(demo01_memory_partition demo01_memory_partition.cpp)
add_executable(demo02_quote demo02_quote.cpp)
add_executable(demo03_functionsPlus demo03_functionsPlus.cpp)
add_executable(demo04_class_packaging demo04_class_packaging.cpp)
add_executable(demo05_initialization demo05_initialization.cpp)
add_executable(demo06_friend demo06_friend.cpp)
add_executable(demo07_operatorOverloading demo07_operatorOverloading.cpp)
add_executable(demo08_inheritance demo08_inheritance.cpp)
add_executable(demo09_multimode demo09_multimode.cpp)
add_executable(demo10_fileIO demo10_fileIO.cpp)
add_executable(demo11_functionTemplate demo11_functionTemplate.cpp)
add_executable(demo12_classTemplate demo12_classTemplate.cpp)
add_executable(demo13_EmployeeManagementSystem 
                    Employee_Management_System/Worker.cpp
                    Employee_Management_System/workerManager.cpp 
                    Employee_Management_System/demo13_EmployeeManagementSystem.cpp)