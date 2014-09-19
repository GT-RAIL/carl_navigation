cmake_minimum_required(VERSION 2.8.3)
project(carl_navigation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp actionlib move_base_msgs std_msgs geometry_msgs roslib actionlib_msgs message_generation)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate actions in the 'action' folder
add_action_files(
  FILES
  MoveCarl.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
  geometry_msgs
)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(rail_lab_location_server src/rail_lab_location_server.cpp src/location.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(rail_lab_location_server
  ${catkin_LIBRARIES}
  yaml-cpp
)

add_dependencies(rail_lab_location_server ${PROJECT_NAME}_gencpp)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS rail_lab_location_server
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Copy header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

## Copy launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## Copy rviz files
install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)

## Copy config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
