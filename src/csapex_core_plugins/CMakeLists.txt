cmake_minimum_required(VERSION 2.8.3)
project(csapex_core_plugins)

find_package(catkin REQUIRED COMPONENTS roscpp utils utils_plugin csapex)

find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})

set(CMAKE_BUILD_TYPE Debug)

file(GLOB_RECURSE MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.h)
QT4_ADD_RESOURCES(QT_RESOURCES res/csapex_core_resources.qrc)
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
  CATKIN_DEPENDS csapex
#  DEPENDS
)

###########
## Build ##
###########

include_directories(include
    ${catkin_INCLUDE_DIRS}
)

add_library(csapex_core_plugins
    src/double_buffer.cpp
    src/export_cout.cpp
    src/file_importer.cpp
    src/import_cin.cpp
    src/import_ros.cpp
    src/ros_message_conversion.cpp
    src/global_option_panel.cpp
    src/text_input.cpp
    src/string_message.cpp
    src/register_core_plugins.cpp
    ${QT_RESOURCES}
    ${MOC}
)

target_link_libraries(csapex_core_plugins
    ${catkin_LIBRARIES}
)
