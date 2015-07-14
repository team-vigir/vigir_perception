cmake_minimum_required(VERSION 2.8.3)
project(vigir_crop_decimate_nodelet)

find_package(catkin REQUIRED COMPONENTS vigir_crop_decimate vigir_perception_msgs nodelet camera_info_manager roscpp)

## Declare things to be passed to other projects
catkin_package(
    LIBRARIES ${PROJECT_NAME} #${PROJECT_NAME}_requester
    CATKIN_DEPENDS vigir_crop_decimate vigir_perception_msgs nodelet camera_info_manager roscpp
)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

#common commands for building c++ executables and libraries
#Libraries
add_library(${PROJECT_NAME} src/crop_decimate_nodelet.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

add_library(${PROJECT_NAME}_requester src/crop_decimate_requester_nodelet.cpp)
target_link_libraries(${PROJECT_NAME}_requester ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_requester ${catkin_EXPORTED_TARGETS})

# Executables
add_executable(crop_decimate_node src/crop_decimate_node.cpp)
target_link_libraries(crop_decimate_node ${PROJECT_NAME}  ${catkin_LIBRARIES})
add_dependencies(crop_decimate_node ${catkin_EXPORTED_TARGETS})

add_executable(crop_decimate_requester_node src/crop_decimate_requester_node.cpp)
target_link_libraries(crop_decimate_requester_node ${PROJECT_NAME}_requester  ${catkin_LIBRARIES})
add_dependencies(crop_decimate_requester_node ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME} crop_decimate_node ${PROJECT_NAME}_requester crop_decimate_requester_node
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY plugin_descriptions
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.*"

)
