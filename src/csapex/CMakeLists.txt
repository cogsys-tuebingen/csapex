cmake_minimum_required(VERSION 2.8.3)
project(csapex)

set(CMAKE_BUILD_TYPE Debug)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

add_definitions(-W -Wall)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS utils_plugin)

find_package(Boost REQUIRED)
find_package(Boost COMPONENTS program_options filesystem system regex REQUIRED)

find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})


file(GLOB         QT_FORMS    RELATIVE   ${CMAKE_CURRENT_SOURCE_DIR}                 ui/*.ui)
file(GLOB_RECURSE QT_SOURCES  RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}   FOLLOW_SYMLINKS src/*.h include/*.hpp include/*.h)

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
   CATKIN_DEPENDS utils_plugin
   DEPENDS Qt4
)

###########
## Build ##
###########

include_directories(include
    ${catkin_INCLUDE_DIRS}
)

##### EVALUATION
add_library(csapex SHARED
    src/core/core_plugin.cpp
    src/core/csapex_core.cpp
    src/core/graphio.cpp
    src/core/designerio.cpp
    src/core/drag_io.cpp

    src/command/command.cpp
    src/command/dispatcher.cpp
    src/command/meta.cpp
    src/command/add_node.cpp
    src/command/add_connection.cpp
    src/command/add_connector.cpp
    src/command/move_connection.cpp
    src/command/delete_node.cpp
    src/command/delete_connection.cpp
    src/command/delete_connector.cpp
    src/command/move_box.cpp
    src/command/add_fulcrum.cpp
    src/command/move_fulcrum.cpp
    src/command/delete_fulcrum.cpp

    src/model/group.cpp
    src/model/boxed_object.cpp
    src/model/boxed_object_constructor.cpp
    src/model/error_state.cpp
    src/model/graph.cpp
    src/model/connection.cpp
    src/model/connector.cpp
    src/model/connector_in.cpp
    src/model/connector_out.cpp
    src/model/connector_forward.cpp
    src/model/connection_type.cpp
    src/model/memento.cpp
    src/model/message.cpp
    src/model/message_provider.cpp
    src/model/node.cpp
    src/model/node_state.cpp
    src/model/node_worker.cpp
    src/model/tag.cpp
    src/model/template.cpp
    src/model/template_constructor.cpp

    src/manager/box_manager.cpp
    src/manager/connection_type_manager.cpp
    src/manager/message_provider_manager.cpp
    src/manager/template_manager.cpp

    src/view/box.cpp
    src/view/csapex_window.cpp
    src/view/design_board.cpp
    src/view/designer.cpp
    src/view/node_adapter.cpp
    src/view/overlay.cpp
    src/view/selectable.cpp
    src/view/template_dialog.cpp
    src/view/profiling_widget.cpp

    src/utility/bash_parser.cpp
    src/utility/qdouble_slider.cpp
    src/utility/qsignal_bridges.cpp
    src/utility/qwrapper.cpp
    src/utility/stream_interceptor.cpp
    src/utility/timer.cpp
    include/csapex/utility/qt_helper.hpp

    src/deprecated/global_option.cpp
    src/deprecated/global_option_manager.cpp
    ${QT}
)

message(${catkin_LIBRARIES})

target_link_libraries(csapex
    yaml-cpp
    ${QT_LIBRARIES}
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)

add_executable(csapex_node
    src/csapex.cpp)
target_link_libraries(csapex_node csapex)

execute_process(COMMAND ./setup.sh ${CATKIN_DEVEL_PREFIX} ${CATKIN_PACKAGE_BIN_DESTINATION} csapex_node WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR})
