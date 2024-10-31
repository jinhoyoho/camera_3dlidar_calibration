cmake_minimum_required(VERSION 3.0.2)
project(camera_3dlidar_calibration)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  message_generation
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  std_msgs
  sensor_msgs
  image_transport
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  obj_info.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}
  LIBRARIES camera_3dlidar_calibration
  CATKIN_DEPENDS cv_bridge pcl_conversions pcl_ros roscpp rospy std_msgs sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(calibration src/calibration.cpp)
target_link_libraries(calibration 
  ${catkin_LIBRARIES}  # catkin 패키지의 라이브러리 링크
  ${OpenCV_LIBRARIES}  # OpenCV 라이브러리 링크
  ${PCL_LIBRARIES}     # PCL 라이브러리 링크
)

add_executable(lidar_pre src/LiDAR_pre.cpp)
target_link_libraries(lidar_pre 
  ${catkin_LIBRARIES}  # catkin 패키지의 라이브러리 링크
  ${PCL_LIBRARIES}     # PCL 라이브러리 링크
)
