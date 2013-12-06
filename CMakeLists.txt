cmake_minimum_required(VERSION 2.8.3)
project(cameraCalibration)
find_package( OpenCV REQUIRED )
include_directories(include)
add_executable(cameraCalibrator src/stereoCameraCalibrator.cpp
								   src/monocularCameraCalibrator.cpp
								   stereoCameraCalibrator.h
								   monocularCameraCalibrator.h)
target_link_libraries(cameraCalibrator ${OpenCV_LIBS} )	
