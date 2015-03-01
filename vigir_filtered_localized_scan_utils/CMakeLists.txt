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
target_link_libraries(scan_to_clouds_pub_node
   ${catkin_LIBRARIES}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")

install(TARGETS scan_to_clouds_pub_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
