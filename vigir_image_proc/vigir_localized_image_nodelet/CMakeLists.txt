cmake_minimum_required(VERSION 3.0.2)
project(vigir_localized_image_nodelet)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED  cv_bridge image_geometry image_transport nodelet roscpp sensor_msgs dynamic_reconfigure tf tf_conversions vigir_localized_image_proc)
#find_package(OpenCV REQUIRED core highgui imgproc)
find_package(Boost REQUIRED COMPONENTS thread)

#generate_dynamic_reconfigure_options(cfg/Rectify.cfg)


## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS image_geometry roscpp sensor_msgs tf tf_conversions vigir_localized_image_proc
  INCLUDE_DIRS
  LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
#include_directories(include)

# Nodelet library
add_library(${PROJECT_NAME} src/nodelets/rectify.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# install the launch file
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#install the launch files.
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch"  
)
