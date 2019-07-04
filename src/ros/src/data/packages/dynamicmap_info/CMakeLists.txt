cmake_minimum_required(VERSION 2.8.3)
project(dynamicmap_info)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gnss
  jsk_recognition_msgs
  tf
  autoware_msgs
)

set(CMAKE_CXX_FLAGS "-std=c++14 -O2 -Wall ${CMAKE_CXX_FLAGS}")

###################################
## catkin specific configuration ##
###################################
catkin_package(
   CATKIN_DEPENDS roscpp autoware_msgs
   DEPENDS ssh2 xerces-c
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(dynamicmap_info
#  lib/dynamicmap_info/SendData.cpp
  lib/dynamicmap_info/util.cpp
)

add_executable(dynamicmap_info_downloader nodes/dynamicmap_info_downloader/dynamicmap_info_downloader.cpp)
target_link_libraries(dynamicmap_info_downloader ${catkin_LIBRARIES} dynamicmap_info is xerces-c pthread)

add_executable(dynamicmap_info_uploader nodes/dynamicmap_info_uploader/dynamicmap_info_uploader.cpp)
target_link_libraries(dynamicmap_info_uploader ${catkin_LIBRARIES} dynamicmap_info is ssh2 xerces-c)
add_dependencies(dynamicmap_info_uploader autoware_msgs_generate_messages_cpp)

#set(CAMERA_YAML "$ENV{HOME}/.ros/autoware/camera_lidar_2d.yaml")

#set_target_properties(dynamicmap_info_uploader
#  PROPERTIES COMPILE_FLAGS
#  "-DCAMERA_YAML=${CAMERA_YAML}")

