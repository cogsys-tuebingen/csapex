cmake_minimum_required(VERSION 2.8.3)
project(vision_core_plugins)

find_package(catkin REQUIRED COMPONENTS roscpp vision_evaluator utils)

find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})

set(CMAKE_BUILD_TYPE Debug)

file(GLOB_RECURSE MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.h)
QT4_ADD_RESOURCES(QT_RESOURCES res/vision_core_resources.qrc)
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
#  LIBRARIES
#  CATKIN_DEPENDS
#  DEPENDS
)

###########
## Build ##
###########

include_directories(include
    ${catkin_INCLUDE_DIRS}
)

add_library(vision_core_plugins
    src/double_buffer.cpp
    src/export_cout.cpp
    src/import_cin.cpp
    src/import_ros.cpp
    src/ros_message_conversion.cpp
    src/file_importer.cpp
    src/filter_blur.cpp
    src/filter_apply_mask.cpp
    src/filter_static_mask.cpp
    src/global_option_panel.cpp
    src/image_combiner_robust_match.cpp
    src/output_display.cpp
    src/text_input.cpp
    ${QT_RESOURCES}
    ${MOC}
)

target_link_libraries(vision_core_plugins
    ${catkin_LIBRARIES}
)
