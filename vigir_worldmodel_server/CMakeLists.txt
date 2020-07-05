cmake_minimum_required(VERSION 2.8.3)
project(vigir_worldmodel_server)

find_package(catkin REQUIRED octomap_msgs octomap_ros pcl_ros vigir_perception_msgs hector_nav_msgs hector_std_msgs)
link_libraries(${OCTOMAP_LIBRARIES})
find_package(PCL REQUIRED)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

#link_directories(${OCTOMAP_LIBRARY_DIRS})

add_definitions(-DOCTOMAP_NODEBUGOUT)

file(GLOB_RECURSE HEADER_FILES "include/*.h")


## Declare things to be passed to other projects
catkin_package(
    INCLUDE_DIRS include
    #LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp std_msgs octomap_msgs octomap_ros pcl_ros vigir_perception_msgs hector_nav_msgs
)

#common commands for building c++ executables and libraries
add_executable(worldmodel_server_node src/worldmodel_server_node.cpp ${HEADER_FILES})
target_link_libraries(worldmodel_server_node ${catkin_LIBRARIES})
add_dependencies(worldmodel_server_node ${catkin_EXPORTED_TARGETS})

install(TARGETS worldmodel_server_node
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#install the launch files.
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch"
)
