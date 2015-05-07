cmake_minimum_required(VERSION 2.8.3)
project(vigir_localized_image_proc)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
find_package(catkin REQUIRED COMPONENTS sensor_msgs cv_bridge image_transport)

SET(HEADERS
include/${PROJECT_NAME}/localized_image_proc.h
)

SET(SOURCE
src/localized_image_proc.cpp
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

## Declare things to be passed to other projects
catkin_package(
    LIBRARIES ${PROJECT_NAME}
    INCLUDE_DIRS include
    CATKIN_DEPENDS sensor_msgs cv_bridge image_transport
    DEPENDS opencv boost 
)


add_library(${PROJECT_NAME} ${HEADERS} ${SOURCE})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

#install execs
install(TARGETS ${PROJECT_NAME}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

