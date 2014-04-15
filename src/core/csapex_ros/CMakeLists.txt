cmake_minimum_required(VERSION 2.8.3)
project(csapex_ros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED roscpp csapex csapex_core_plugins)

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${catkin_INCLUDE_DIRS})

## Declare a cpp library
add_library(csapex_ros
  src/apex_ros_interface.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(csapex_ros
  ${catkin_LIBRARIES}
)

