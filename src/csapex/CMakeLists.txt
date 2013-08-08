cmake_minimum_required(VERSION 2.8.3)
project(csapex)

set(CMAKE_BUILD_TYPE Debug)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS pluginlib roscpp rospy utils utils_vision utils_plugin)

find_package(Boost REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)

find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})


file(GLOB         QT_FORMS    RELATIVE   ${CMAKE_CURRENT_SOURCE_DIR}                 ui/*.ui)
file(GLOB_RECURSE QT_SOURCES  RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}   FOLLOW_SYMLINKS src/*.h include/*.h)

QT4_WRAP_CPP(QT_SOURCES ${QT_SOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_ADD_RESOURCES(QT_RESOURCES res/csapex_resources.qrc)

set(QT ${QT_SOURCES} ${QT_FORMS_HPP} ${QT_RESOURCES})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${catkin_INCLUDE_DIRS})

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES csapex
   CATKIN_DEPENDS pluginlib roscpp
   DEPENDS Qt4
)

###########
## Build ##
###########


##### EVALUATION
add_library(csapex SHARED
    src/box.cpp
    src/boxed_object.cpp
    src/box_manager.cpp
    src/core_plugin.cpp
    src/command.cpp
    src/command_dispatcher.cpp
    src/command_meta.cpp
    src/command_add_box.cpp
    src/command_add_connection.cpp
    src/command_move_connection.cpp
    src/command_delete_box.cpp
    src/command_delete_connection.cpp
    src/command_delete_connector.cpp
    src/command_move_box.cpp
    src/connection.cpp
    src/connector.cpp
    src/connector_in.cpp
    src/connector_out.cpp
    src/connection_type.cpp
    src/connection_type_manager.cpp
    src/csapex_window.cpp
    src/design_board.cpp
    src/designer.cpp
    src/designerio.cpp
    src/displayable.cpp
    src/global_option.cpp
    src/global_option_manager.cpp
    src/graph.cpp
    src/graphio.cpp
    src/message.cpp
    src/message_provider.cpp
    src/message_provider_manager.cpp
    src/memento.cpp
    src/overlay.cpp
    src/selector_proxy.cpp
    src/selectable.cpp
    src/stream_interceptor.cpp
    src/qdouble_slider.cpp
    src/qwrapper.cpp
    src/qsignal_bridges.cpp
    src/bash_parser.cpp

    include/csapex/qt_helper.hpp
    ${QT}
)
target_link_libraries(csapex
    yaml-cpp ${QT_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_executable(csapex_node
    src/csapex.cpp)
target_link_libraries(csapex_node csapex)
