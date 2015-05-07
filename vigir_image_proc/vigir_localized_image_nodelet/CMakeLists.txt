cmake_minimum_required(VERSION 2.8.3)
project(vigir_localized_image_nodelet)

find_package(catkin REQUIRED COMPONENTS vigir_localized_image_proc flor_perception_msgs nodelet camera_info_manager roscpp)

## Declare things to be passed to other projects
catkin_package(
    LIBRARIES ${PROJECT_NAME} #${PROJECT_NAME}_requester
    CATKIN_DEPENDS vigir_localized_image_proc flor_perception_msgs nodelet camera_info_manager roscpp tf
)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

#common commands for building c++ executables and libraries
#Libraries
add_library(localized_image_nodelet src/localized_image_nodelet.cpp)
target_link_libraries(localized_image_nodelet ${catkin_LIBRARIES})
add_dependencies(localized_image_nodelet ${catkin_EXPORTED_TARGETS})

# Executables
add_executable(localized_image_node src/localized_image_node.cpp)
target_link_libraries(localized_image_node localized_image_nodelet  ${catkin_LIBRARIES})
add_dependencies(localized_image_node ${catkin_EXPORTED_TARGETS})

install(TARGETS localized_image_nodelet localized_image_node
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY plugin_descriptions
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.*"

)
