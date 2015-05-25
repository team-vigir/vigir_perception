cmake_minimum_required(VERSION 2.8.3)
project(vigir_point_cloud_proc)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
  shape_msgs
  pcl_msgs
  visualization_msgs
  image_transport
  image_geometry
  depth_image_proc
  tf
  tf_conversions
)

find_package(PCL REQUIRED)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vigir_point_cloud_proc
  CATKIN_DEPENDS pcl_ros roscpp sensor_msgs shape_msgs visualization_msgs pcl_msgs image_transport image_geometry depth_image_proc tf tf_conversions
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#common commands for building c++ executables and libraries
add_executable(cloud_to_mesh_node src/cloud_to_mesh_node.cpp ${HEADER_FILES})
target_link_libraries(cloud_to_mesh_node ${catkin_LIBRARIES})
add_dependencies(cloud_to_mesh_node ${catkin_EXPORTED_TARGETS})

add_executable(depth_image_to_mesh_node src/depth_image_to_mesh_node.cpp ${HEADER_FILES})
target_link_libraries(depth_image_to_mesh_node ${catkin_LIBRARIES})
add_dependencies(depth_image_to_mesh_node ${catkin_EXPORTED_TARGETS})

install(TARGETS cloud_to_mesh_node depth_image_to_mesh_node
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#install the launch files.
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch"
)
