cmake_minimum_required(VERSION 2.8.3)
project(vigir_wide_angle_image_prok)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED  cv_bridge image_geometry image_transport nodelet roscpp sensor_msgs dynamic_reconfigure vigir_ocamlib_tools tf tf_conversions)
#find_package(OpenCV REQUIRED core highgui imgproc)
find_package(Boost REQUIRED COMPONENTS thread)

generate_dynamic_reconfigure_options(cfg/Rectify.cfg)


## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS image_geometry roscpp sensor_msgs vigir_ocamlib_tools tf tf_conversions
  DEPENDS OpenCV
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
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
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

# install the calibration file
install(DIRECTORY calib
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#install the launch files.
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch"  
)
