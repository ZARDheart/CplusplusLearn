set(CMAKE_BUILD_TYPE Release)

find_package(GTSAM REQUIRED) # Uses installed package
find_package(Boost REQUIRED COMPONENTS filesystem program_options system timer date_time thread serialization chrono)

include_directories(
	${GTSAM_INCLUDE_DIR}
	${Boost_INCLUDE_DIRS}
)

link_directories(${GTSAM_LIBRARY_DIRS})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin/gtsam_examples)

add_executable(demo0_test demo0_test.cpp)
target_link_libraries(demo0_test gtsam)

add_executable(demo01_example2D demo01_example2D.cpp)
target_link_libraries(demo01_example2D gtsam)

add_executable(demo02_CustomPoint2Example demo02_CustomPoint2Example.cpp)
target_link_libraries(demo02_CustomPoint2Example gtsam)

add_executable(demo03_example2DWithGPS demo03_example2DWithGPS.cpp)
target_link_libraries(demo03_example2DWithGPS gtsam)

add_executable(demo04_VisualFactor demo04_VisualFactor.cpp)
target_link_libraries(demo04_VisualFactor gtsam)

add_executable(demo05_VisualISAMExample demo05_VisualISAMExample.cpp)
target_link_libraries(demo05_VisualISAMExample gtsam)

add_executable(demo06_VisualISAM2Example demo06_VisualISAM2Example.cpp)
target_link_libraries(demo06_VisualISAM2Example gtsam)

add_executable(demo07_easyPoint2KalmanFilter demo07_easyPoint2KalmanFilter.cpp)
target_link_libraries(demo07_easyPoint2KalmanFilter gtsam)

# add_executable(demo08_elaboratePoint2KalmanFilter demo08_elaboratePoint2KalmanFilter.cpp)
# target_link_libraries(demo08_elaboratePoint2KalmanFilter gtsam)
