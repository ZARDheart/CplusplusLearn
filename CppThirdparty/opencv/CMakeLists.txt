#添加c++ 11标准支持
set(CMAKE_CXX_FLAGS "-std=c++11 -O2")

# Eigen
include_directories("/usr/include/eigen3")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin/opencv)

add_executable(demo1_datatype demo1_datatype.cpp)
target_link_libraries(demo1_datatype ${OpenCV_LIBS} )

add_executable(demo2_ReadWritSaveIm demo2_ReadWritSaveIm.cpp)
target_link_libraries(demo2_ReadWritSaveIm ${OpenCV_LIBS} )

add_executable(demo3_opencvideo demo3_videodemo.cpp)
target_link_libraries(demo3_opencvideo ${OpenCV_LIBS} )

add_executable(demo4_imPlot demo4_imPlot.cpp)
target_link_libraries(demo4_imPlot ${OpenCV_LIBS} )

add_executable(demo5_window demo5_window.cpp)
target_link_libraries(demo5_window ${OpenCV_LIBS} )

add_executable(demo6_keyP_Fea_Match demo6_keyP_Fea_Match.cpp)
target_link_libraries(demo6_keyP_Fea_Match ${OpenCV_LIBS} )

add_executable(demo6_1_ORB demo6_1_ORB.cpp)
target_link_libraries(demo6_1_ORB ${OpenCV_LIBS} )

add_executable(demo7_CameraCalibration demo7_CameraCalibration.cpp)
target_link_libraries(demo7_CameraCalibration ${OpenCV_LIBS} )

add_executable(demo7_1_TheroyUndistortimage demo7_1_TheroyUndistortimage.cpp)
target_link_libraries(demo7_1_TheroyUndistortimage ${OpenCV_LIBS} )

add_executable(demo7_2_StereoCalibrate demo7_2_StereoCalibrate.cpp)
target_link_libraries(demo7_2_StereoCalibrate ${OpenCV_LIBS} )

add_executable(demo8_DeepLeaening demo8_DeepLeaening.cpp)
target_link_libraries(demo8_DeepLeaening ${OpenCV_LIBS} )

find_package(Pangolin REQUIRED)
add_executable(demo9_stereovision demo9_stereovision.cc )
target_link_libraries(demo9_stereovision ${OpenCV_LIBS}  ${Pangolin_LIBRARIES})

add_executable(demo10_BialteralFilter demo10_BialteralFilter.cc)
target_link_libraries(demo10_BialteralFilter ${OpenCV_LIBS} )

add_executable(demo11_myXMLandYMAL demo11_myXMLandYMAL.cpp)
target_link_libraries(demo11_myXMLandYMAL ${OpenCV_LIBS} )
