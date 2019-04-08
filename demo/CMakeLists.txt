cmake_minimum_required (VERSION 2.8)

project(TestIndem)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(DISPLAY_POINT_CLOUD "build point cloud display window" OFF)
if(${DISPLAY_POINT_CLOUD})
add_definitions(-DDISPLAY_POINT_CLOUD)
# find_package(OpenCV)
include_directories(${CMAKE_SOURCE_DIR}/include)
endif()

include_directories(../include)

add_definitions(-std=c++11)
add_executable(${PROJECT_NAME} Indem.cpp)
target_link_libraries(${PROJECT_NAME}
${CMAKE_SOURCE_DIR}/../lib/1604/libindem.so
${CMAKE_SOURCE_DIR}/../lib/1604/libslam.so
${CMAKE_SOURCE_DIR}/../lib/1604/libslam_imp.so
${CMAKE_SOURCE_DIR}/../lib/1604/libusbdriver.so
pthread
stdc++fs
gomp
z
dl
${CMAKE_SOURCE_DIR}/../lib/1604/libtbb.so.2
${CMAKE_SOURCE_DIR}/../lib/1604/libg3logger.so.1.3.0-0
${CMAKE_SOURCE_DIR}/../lib/1604/libprotobuf.so.14
${CMAKE_SOURCE_DIR}/../lib/1604/libnanomsg.so.5
${CMAKE_SOURCE_DIR}/../lib/1604/libopencv_core.so.3.4
${CMAKE_SOURCE_DIR}/../lib/1604/libopencv_flann.so.3.4
${CMAKE_SOURCE_DIR}/../lib/1604/libopencv_photo.so.3.4
${CMAKE_SOURCE_DIR}/../lib/1604/libopencv_calib3d.so.3.4
${CMAKE_SOURCE_DIR}/../lib/1604/libopencv_videoio.so.3.4
${CMAKE_SOURCE_DIR}/../lib/1604/libopencv_highgui.so.3.4
${CMAKE_SOURCE_DIR}/../lib/1604/libopencv_imgproc.so.3.4
${CMAKE_SOURCE_DIR}/../lib/1604/libopencv_imgcodecs.so.3.4
${CMAKE_SOURCE_DIR}/../lib/1604/libopencv_features2d.so.3.4
)
if(${DISPLAY_POINT_CLOUD})
target_link_libraries(${PROJECT_NAME}
  ${CMAKE_SOURCE_DIR}/../lib/1604/libopencv_viz.so.3.4
)
# target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
endif()
