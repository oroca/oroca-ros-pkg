cmake_minimum_required(VERSION 2.8.3)
project(openni2_xtion)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs cv_bridge image_transport camera_info_manager)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs cv_bridge image_transport camera_info_manager
  DEPENDS system_lib
)

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(openni2_xtion src/openni2_xtion.cpp)

target_link_libraries(openni2_xtion ${catkin_LIBRARIES} OpenNI2)

install(TARGETS openni2_xtion
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})


