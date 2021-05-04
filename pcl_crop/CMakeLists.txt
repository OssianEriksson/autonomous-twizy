cmake_minimum_required(VERSION 3.0.2)
project(pcl_crop)

add_compile_options(-std=c++11 -O3 -DNDEBUG)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
  sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(pcl_crop
  src/pcl_crop.cpp
)

add_library(crop_nodelet
  src/pcl_crop_nodelet.cpp
)
target_link_libraries(crop_nodelet pcl_crop) 

install(TARGETS crop_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

message("test: ${CATKIN_GLOBAL_SHARE_DESTINATION}/${PROJECT_NAME}")
install(FILES nodelet_plugins.xml DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/${PROJECT_NAME})