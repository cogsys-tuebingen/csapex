cmake_minimum_required(VERSION 2.8.3)
project(cip104)

find_package(catkin REQUIRED COMPONENTS roscpp vision_evaluator)

find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})

set(CMAKE_BUILD_TYPE Debug)

file(GLOB_RECURSE MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.h)
QT4_WRAP_CPP(MOC ${MOC})

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES cip104
#  CATKIN_DEPENDS roscpp vision_evaluator
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(cip104
  src/filter_hsv_segmentation.cpp
  src/filter_lk_tracking.cpp
  src/virtual_camera.cpp
  ${MOC}
)
target_link_libraries(cip104
  ${catkin_LIBRARIES}
)
