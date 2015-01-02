cmake_minimum_required(VERSION 2.8.11)
project(csapex)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "build type, options: None Debug Release RelWithDebInfo MinSizeRel." FORCE)
    message("No CMAKE_BUILD_TYPE specified, defaulting to ${CMAKE_BUILD_TYPE}")
endif(NOT CMAKE_BUILD_TYPE)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

add_definitions(-W -Wall)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS utils_param utils_qt class_loader)

find_package(Boost COMPONENTS program_options filesystem system regex REQUIRED)

include(cmake/csapex-extras.cmake)

set(YAML_LIBRARIES -L/usr/local/lib/ yaml-cpp)

find_package(OpenGL REQUIRED)
find_package(TinyXML REQUIRED)
find_package(Qt5 COMPONENTS OpenGL REQUIRED)
find_package(console_bridge REQUIRED)

qt5_add_resources(QT_RESOURCES res/csapex_resources.qrc)
qt5_wrap_ui(QT_UI
    ui/box.ui
    ui/csapex_window.ui
    ui/design_board.ui
    ui/designer.ui)

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
   CATKIN_DEPENDS utils_param utils_qt
   DEPENDS Boost console_bridge
   CFG_EXTRAS csapex-extras.cmake
)

###########
## INFO  ##
###########
# Get the current working branch
execute_process(
  COMMAND git rev-parse --abbrev-ref HEAD
  WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
  OUTPUT_VARIABLE GIT_BRANCH
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Get the latest abbreviated commit hash of the working branch
execute_process(
  COMMAND git log -1 --format=%h
  WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
  OUTPUT_VARIABLE GIT_COMMIT_HASH
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

add_definitions("-DGIT_COMMIT_HASH=${GIT_COMMIT_HASH}")
add_definitions("-DGIT_BRANCH=${GIT_BRANCH}")
add_definitions("-DMAINTAINER=${csapex_MAINTAINER}")
add_definitions("-DVERSION=${csapex_VERSION}")
add_definitions("-DCSAPEX_BOOT_PLUGIN_DIR=\"${CSAPEX_BOOT_PLUGIN_DIR}\"")

###########
## Build ##
###########

include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${OPENGL_INCLUDE_DIR}
)

#
# SINGLE OUT PLUGIN RELEVANT FILES
#
set(SRC_ESSENTIAL
    src/core/bootstrap_plugin.cpp
)

set(SRC_PLUGIN
    src/core/core_plugin.cpp

    src/model/node.cpp
    src/model/node_modifier.cpp
    src/model/connection_type.cpp

    src/msg/message.cpp
    src/msg/message_factory.cpp
    src/msg/message_renderer.cpp
    src/msg/input.cpp
    src/msg/output.cpp

    src/signal/signal.cpp
    src/signal/trigger.cpp
    src/signal/slot.cpp

    src/manager/message_provider_manager.cpp
    src/manager/message_renderer_manager.cpp

    src/view/node_adapter.cpp
    src/view/node_adapter_builder.cpp
    src/view/default_node_adapter.cpp
)

set(SRC_APP
    src/plugin/plugin_loader.cpp
    src/plugin/plugin_locator.cpp

    src/model/node_worker.cpp
    src/model/graph.cpp
    src/model/graph_worker.cpp
    src/model/connectable.cpp
    src/model/generic_state.cpp
    src/model/connection.cpp
    src/model/tag.cpp
    src/model/node_constructor.cpp
    src/model/error_state.cpp
    src/model/fulcrum.cpp
    src/model/multi_connection_type.cpp
    src/model/node_factory.cpp
    src/model/node_state.cpp
    src/model/node_statistics.cpp
    src/model/parameterizable.cpp
    src/model/node_filter_proxy_model.cpp
    src/model/unique.cpp
    src/model/memento.cpp

    src/msg/apex_message_provider.cpp

    src/core/csapex_core.cpp
    src/core/graphio.cpp
    src/core/designerio.cpp
    src/core/drag_io.cpp
    src/core/settings.cpp
    src/core/thread_pool.cpp

    src/msg/message_provider.cpp

    src/factory/generic_node_factory.cpp

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
    src/command/modify_fulcrum.cpp
    src/command/delete_fulcrum.cpp
    src/command/flip_sides.cpp
    src/command/minimize.cpp
    src/command/create_thread.cpp
    src/command/switch_thread.cpp


    src/utility/error_handling.cpp
    src/utility/timable.cpp
    src/utility/type.cpp
    src/utility/q_signal_relay.cpp
    src/utility/context_menu_handler.cpp
    src/utility/stream_interceptor.cpp
    src/utility/stream_relay.cpp
    src/utility/bash_parser.cpp
    src/utility/buffer.cpp
    src/utility/movable_graphics_proxy_widget.cpp
    src/utility/html_delegate.cpp
    src/utility/yaml_node_builder.cpp
    src/utility/qint_slider.cpp
    src/utility/qdouble_slider.cpp
    src/utility/qsignal_bridges.cpp
    src/utility/qwrapper.cpp
    src/utility/timer.cpp
    src/utility/qt_helper.cpp
    src/utility/thread.cpp
    src/utility/uuid.cpp
    src/utility/assert.cpp
    src/utility/widget_picker.cpp

    src/view/box.cpp
    src/view/box_dialog.cpp
    src/view/csapex_window.cpp
    src/view/designer_scene.cpp
    src/view/designer_view.cpp
    src/view/designer.cpp
    src/view/node_adapter_factory.cpp
    src/view/profiling_widget.cpp
    src/view/port.cpp
    src/view/widget_controller.cpp
    src/view/fulcrum_widget.cpp
    src/view/fulcrum_handle.cpp
    src/view/parameter_context_menu.cpp
    src/view/activity_timeline.cpp
    src/view/activity_legend.cpp

    ${QT_RESOURCES}

    ${QT_UI}
)


#
# BUILD THE LIBRARIES
#

add_library(csapex_core SHARED
    ${SRC_ESSENTIAL}
)


add_library(csapex_pluginbase SHARED
    ${SRC_PLUGIN}
)

target_link_libraries(csapex_pluginbase
    csapex_core
    ${QT_LIBRARIES}
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    ${YAML_LIBRARIES}
    ${TinyXML_LIBRARIES}
    ${console_bridge_LIBRARIES}
)


add_library(csapex_app SHARED
    ${SRC_APP}
    ${QT_FORMS_HPP}
)

target_link_libraries(csapex_app csapex_pluginbase
    ${QT_LIBRARIES}
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${YAML_LIBRARIES}
    ${console_bridge_LIBRARIES}
    Qt5::OpenGL
)

#
# LIST EVERYTHING IN QT
#
file(GLOB_RECURSE ALL_HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/*.hpp include/*.h)


#
# BUILD THE APPLICATION
#

add_executable(csapex_node
    ${ALL_HEADERS}

    src/csapex.cpp)
target_link_libraries(csapex_node csapex_app)


add_custom_target(cfg ALL DEPENDS cfg/style.css)
add_custom_command(TARGET cfg POST_BUILD
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/setup.sh
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
)

#
# TESTS
#
set(CSAPEX_TESTING 0 CACHE BOOL "build tests?")
message("BUILD TESTS? " ${CSAPEX_TESTING})
if(${CSAPEX_TESTING})
    enable_testing()

    set(TESTS
        tests/tests.cpp

        tests/uuid_test.cpp
        tests/graph_test.cpp
        tests/node_creation_test.cpp
    )

    # UNIT TEST
    add_executable(csapex_run_unit_tests ${TESTS})
    target_link_libraries(csapex_run_unit_tests gtest gtest_main csapex_app)

    # PROFILING
    add_executable(csapex_profile_unit_tests
        ${ALL_HEADERS}
        ${TESTS}
        ${SRC_ESSENTIAL}
        ${SRC_PLUGIN}
        ${QT_PLUGIN}
        ${SRC_APP}
        ${QT_APP})
    set_target_properties(csapex_profile_unit_tests PROPERTIES COMPILE_FLAGS "--coverage")
    target_link_libraries(csapex_profile_unit_tests gtest gtest_main gcov
        ${QT_LIBRARIES}
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
        ${OPENGL_LIBRARIES}
        ${YAML_LIBRARIES}
        ${console_bridge_LIBRARIES}
        Qt5::OpenGL
    )
    add_test(csapex_profile_unit_tests ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/csapex/csapex_profile_unit_tests)
    add_custom_target(check COMMAND ${CMAKE_CTEST_COMMAND}
                      DEPENDS csapex_profile_unit_tests)
endif()


#
# INSTALL
#

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS csapex_pluginbase
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(TARGETS csapex_app
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(TARGETS csapex_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
