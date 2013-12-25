cmake_minimum_required(VERSION 2.8.3)
project(csapex)

set(CMAKE_BUILD_TYPE Debug)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

add_definitions(-W -Wall)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS utils_plugin utils_param)

find_package(Boost REQUIRED)
find_package(Boost COMPONENTS program_options filesystem system regex REQUIRED)

find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})

file(GLOB   QT_FORMS    RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}  ui/*.ui)
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})

QT4_ADD_RESOURCES(QT_RESOURCES res/csapex_resources.qrc)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

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
   LIBRARIES csapex_pluginbase
   CATKIN_DEPENDS utils_plugin utils_param
   DEPENDS Qt4 Boost
   CFG_EXTRAS csapex-extras.cmake
)

###########
## Build ##
###########

include_directories(include
    ${catkin_INCLUDE_DIRS}
)

#
# SINGLE OUT PLUGIN RELEVANT FILES
#
set(SRC_PLUGIN
    src/core/core_plugin.cpp

    src/model/boxed_object.cpp
    src/model/node.cpp
    src/model/connector.cpp
    src/model/connector_in.cpp
    src/model/connector_out.cpp
    src/model/memento.cpp
    src/model/message.cpp
    src/model/tag.cpp

    src/manager/box_manager.cpp
    src/manager/connection_type_manager.cpp
    src/manager/message_provider_manager.cpp
    src/manager/template_manager.cpp

    src/utility/bash_parser.cpp
    src/utility/qdouble_slider.cpp
    src/utility/qsignal_bridges.cpp
    src/utility/qwrapper.cpp
    src/utility/stream_interceptor.cpp
    src/utility/timer.cpp
    src/utility/qt_helper.cpp

    src/deprecated/global_option.cpp
    src/deprecated/global_option_manager.cpp
)

set(SRC_APP
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
    src/model/node_constructor.cpp
    src/model/error_state.cpp
    src/model/graph.cpp
    src/model/connection.cpp
    src/model/connector_forward.cpp
    src/model/connection_type.cpp
    src/model/generic_state.cpp
    src/model/message_provider.cpp
    src/model/node_state.cpp
    src/model/node_worker.cpp
    src/model/template.cpp
    src/model/template_constructor.cpp
    src/model/node_filter_proxy_model.cpp

    src/view/box.cpp
    src/view/box_dialog.cpp
    src/view/csapex_window.cpp
    src/view/design_board.cpp
    src/view/designer.cpp
    src/view/node_adapter.cpp
    src/view/overlay.cpp
    src/view/selectable.cpp
    src/view/template_dialog.cpp
    src/view/profiling_widget.cpp
)

#
# DETERMINE ALL FILES THAT NEED TO BE MOC'D AND DIVIDE INTO PLUGINBASE / APP
#
file(GLOB_RECURSE QT_SOURCES  RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.h include/*.hpp include/*.h)
set(QT_PLUGIN_MOC)
set(QT_APP_MOC)
foreach(header ${QT_SOURCES})
    file(STRINGS "${header}" lines REGEX "Q_OBJECT")
    if(lines)
        GET_FILENAME_COMPONENT(comp ${header} NAME_WE)
        set(found)
        foreach(plug ${SRC_PLUGIN})
            GET_FILENAME_COMPONENT(comp_plug ${plug} NAME_WE)
            if(${comp_plug} STREQUAL ${comp})
                set(found TRUE)
            endif()
        endforeach()
        if(${found})
            list(APPEND QT_PLUGIN_MOC "${header}")
        else()
            list(APPEND QT_APP_MOC "${header}")
        endif()
    endif()
endforeach()

QT4_WRAP_CPP(QT_PLUGIN_MOC ${QT_PLUGIN_MOC})
QT4_WRAP_CPP(QT_APP_MOC ${QT_APP_MOC})

set(QT_PLUGIN
${QT_PLUGIN_MOC}
${QT_FORMS_HPP} ${QT_RESOURCES})

set(QT_APP
${QT_APP_MOC}
${QT_FORMS_HPP} ${QT_RESOURCES})

#
# BUILD THE LIBRARIES
#

add_library(csapex_pluginbase SHARED
    ${SRC_PLUGIN}
    ${QT_PLUGIN}
)

target_link_libraries(csapex_pluginbase
    yaml-cpp
    ${QT_LIBRARIES}
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)

add_library(csapex_app SHARED
    ${SRC_APP}
    ${QT_APP}
)

target_link_libraries(csapex_app csapex_pluginbase
    yaml-cpp
    ${QT_LIBRARIES}
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)

#
# BUILD THE APPLICATION
#

add_executable(csapex_node
    src/csapex.cpp)
target_link_libraries(csapex_node csapex_app)

execute_process(COMMAND ./setup.sh ${CATKIN_DEVEL_PREFIX} ${CATKIN_PACKAGE_BIN_DESTINATION} csapex_node WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR})
