cmake_minimum_required(VERSION 2.8.3)
project(vigir_filtered_localized_scan_utils)

find_package(catkin REQUIRED COMPONENTS
  laser_geometry
  roscpp
  vigir_perception_msgs
  tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS laser_geometry roscpp vigir_perception_msgs tf
  DEPENDS)


add_executable(scan_to_clouds_pub_node src/scan_to_clouds_pub_node.cpp)
# Surprised below is required, but see https://github.com/raptort3000/ublox_catkin/pull/1
add_dependencies(scan_to_clouds_pub_node ${vigir_perception_msgs_EXPORTED_TARGETS})
target_link_libraries(scan_to_clouds_pub_node
   ${catkin_LIBRARIES}
)

add_executable(scan_split_node src/scan_split_node.cpp)
# Surprised below is required, but see https://github.com/raptort3000/ublox_catkin/pull/1
add_dependencies(scan_split_node ${vigir_perception_msgs_EXPORTED_TARGETS})
target_link_libraries(scan_split_node
   ${catkin_LIBRARIES}
)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")

install(TARGETS scan_to_clouds_pub_node scan_split_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#install the launch files.
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch"
)
