cmake_minimum_required(VERSION 2.8.3)
project(indemind_module)

add_compile_options(-std=c++11)

FIND_PACKAGE(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
  cv_bridge
  tf
  pcl_conversions
  pcl_ros
)
add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY)
find_package(OpenCV 3.4.3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES indemind_module
  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs geometry_msgs tf 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(module_driver src/camera_driver.cpp)
FIND_PACKAGE(Boost 1.58.0 REQUIRED COMPONENTS system filesystem)

add_dependencies(module_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 TARGET_LINK_LIBRARIES(module_driver
   ${PROJECT_SOURCE_DIR}/../../lib/1604/libindem.so
   ${PROJECT_SOURCE_DIR}/../../lib/1604/libboost_filesystem.so.1.58.0
   ${PROJECT_SOURCE_DIR}/../../lib/1604/libboost_system.so.1.58.0
   ${PROJECT_SOURCE_DIR}/../../lib/1604/libg3logger.so.1.3.0-0
   ${PROJECT_SOURCE_DIR}/../../lib/1604/libnanomsg.so.5
   ${PROJECT_SOURCE_DIR}/../../lib/1604/libopencv_core.so.3.4
   ${PROJECT_SOURCE_DIR}/../../lib/1604/libopencv_imgproc.so.3.4
   ${PROJECT_SOURCE_DIR}/../../lib/1604/libopencv_videoio.so.3.4
   ${catkin_LIBRARIES}
   pthread
   stdc++fs
   dl
 )
