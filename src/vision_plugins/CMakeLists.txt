    cmake_minimum_required(VERSION 2.8.3)
project(vision_plugins)

find_package(catkin REQUIRED roscpp csapex_vision utils)

find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})

set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_CXX_FLAGS "-O3")

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
#  LIBRARIES image_transformation
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include
    ${catkin_INCLUDE_DIRS}
)

add_library(vision_plugins
    src/combiner_gridcompare.cpp
    src/combiner_gridcompare_hist.cpp
    src/combiner_gridheatmap_hist.cpp
    src/combiner_gridcompare_value.cpp
    src/combiner_gridheatmap_value.cpp
    src/combiner_set_operation.cpp
    src/filter_merger.cpp
    src/filter_splitter.cpp
    src/filter_equalize.cpp
    src/filter_undistort.cpp
    src/filter_perspective.cpp
    src/filter_histogram.cpp
    src/filter_coloradjustment.cpp
    src/filter_colorconvert.cpp
    src/filter_debayer.cpp
    src/filter_local_patterns.cpp
    src/simple_image_difference.cpp
    src/simple_sink.cpp 
    src/object_detection.cpp
    src/object_homography.cpp
   ${MOC}
)

target_link_libraries(vision_plugins
  ${catkin_LIBRARIES}
)
