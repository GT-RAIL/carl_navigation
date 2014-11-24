cmake_minimum_required(VERSION 2.8.3)
project(carl_navigation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS actionlib actionlib_msgs costmap_2d dynamic_reconfigure geometry_msgs message_generation move_base_msgs nav_msgs rail_ceiling roscpp roslib tf)
find_package(PkgConfig)

#######################################
## Declare ROS messages and services ##
#######################################

add_message_files(
  FILES
  BlockedCells.msg
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  MoveCarl.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  std_msgs
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

## Check for what version of YAML we have
pkg_check_modules(YAML_CPP yaml-cpp)
if(${YAML_CPP_VERSION} VERSION_GREATER 0.5)
  add_definitions(-DYAMLCPP_GT_0_5_0)
endif()

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)
link_directories(${YAML_CPP_LIBRARY_DIRS})

## Declare a cpp executable
add_executable(rail_lab_location_server src/rail_lab_location_server.cpp src/location.cpp)
add_executable(map_republisher src/map_republisher.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(rail_lab_location_server
  ${catkin_LIBRARIES}
  yaml-cpp
)
target_link_libraries(map_republisher ${catkin_LIBRARIES})

add_library(furniture_layer src/furniture_layer.cpp)

add_dependencies(rail_lab_location_server ${PROJECT_NAME}_gencpp)
add_dependencies(furniture_layer ${PROJECT_NAME}_gencpp rail_ceiling_gencpp)
add_dependencies(map_republisher ${PROJECT_NAME}_gencpp)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS rail_lab_location_server map_republisher furniture_layer
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
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
