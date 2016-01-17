cmake_minimum_required(VERSION 2.8.11)

project(csapex)

## Check the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "build type, options: None Debug Release RelWithDebInfo MinSizeRel." FORCE)
    message("No CMAKE_BUILD_TYPE specified, defaulting to ${CMAKE_BUILD_TYPE}")
endif(NOT CMAKE_BUILD_TYPE)

## Add our own cmake subdirectory
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(cmake/csapex-extras.cmake)

## Enable warnings
add_definitions(-W -Wall)

## Enable Profiling
#add_definitions(-ftime-report)

## Find catkin macros and libraries
find_package(catkin REQUIRED)
find_package(class_loader REQUIRED)

find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)
find_package(Boost COMPONENTS program_options filesystem system regex REQUIRED)

set(YAML_LIBRARIES -L/usr/local/lib/ yaml-cpp)

find_package(OpenGL REQUIRED)
find_package(TinyXML REQUIRED)
find_package(Qt5 COMPONENTS OpenGL REQUIRED)
find_package(console_bridge REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

qt5_add_resources(QT_RESOURCES res/csapex_resources.qrc)
qt5_wrap_ui(QT_UI
    ui/box.ui
    ui/csapex_window.ui
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
   # CATKIN_DEPENDS
   DEPENDS Boost console_bridge
   CFG_EXTRAS csapex-extras.cmake
)

#
# CCACHE
#
set(USE_CCACHE 0 CACHE BOOL "optimize compile using ccache?")
if(${USE_CCACHE})
SET_PROPERTY(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
SET_PROPERTY(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif()

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

# generate meta infos
set(INFO_H ${CMAKE_CURRENT_LIST_DIR}/include/csapex/info.h)
set(INFO ${INFO_H}.tmp)

file(WRITE ${INFO} "namespace csapex {\nnamespace info {\n")

file(APPEND ${INFO} "static const std::string GIT_COMMIT_HASH { \"${GIT_COMMIT_HASH}\" };\n")
file(APPEND ${INFO} "static const std::string GIT_BRANCH { \"${GIT_BRANCH}\" };\n")
file(APPEND ${INFO} "static const std::string MAINTAINER { \"${csapex_MAINTAINER}\" };\n")
file(APPEND ${INFO} "static const std::string VERSION { \"${csapex_VERSION}\" };\n")
file(APPEND ${INFO} "static const std::string CSAPEX_BOOT_PLUGIN_DIR { \"${CSAPEX_BOOT_PLUGIN_DIR}\" };\n")
file(APPEND ${INFO} "static const std::string CSAPEX_VERSION { \"${csapex_VERSION}\" };\n")

file(APPEND ${INFO} "}\n}\n\n")

execute_process(COMMAND bash -c "if [[ ! -f info.h  ]] || [[ `diff info.h.tmp info.h` ]]; then mv info.h.tmp info.h; else rm info.h.tmp; fi"
                WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/include/csapex/)

###########
## Build ##
###########

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/external)

include_directories(include

    external/qxt5/include

    ${catkin_INCLUDE_DIRS}
    ${OPENGL_INCLUDE_DIR}
    ${Qt5Core_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS}
)

if (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_definitions(--system-header-prefix=/usr/include)
endif()

#
# BUILD THE LIBRARIES
#

set(csapex_LIBRARIES)

add_library(csapex_param SHARED
    src/param/io.cpp
    src/param/parameter.cpp
    src/param/parameter_builder.cpp
    src/param/parameter_description.cpp

    src/param/range_parameter.cpp
    src/param/interval_parameter.cpp
    src/param/set_parameter.cpp
    src/param/bitset_parameter.cpp
    src/param/value_parameter.cpp
    src/param/path_parameter.cpp
    src/param/trigger_parameter.cpp
    src/param/color_parameter.cpp
    src/param/angle_parameter.cpp
    src/param/null_parameter.cpp
    src/param/string_list_parameter.cpp

    src/param/output_progress_parameter.cpp

    src/param/parameter_factory.cpp
    src/param/parameter_map.cpp
    src/param/parameter_provider.cpp
)
target_link_libraries(csapex_param
    ${Boost_LIBRARIES}
    ${YAML_LIBRARIES}
)

list(APPEND csapex_LIBRARIES csapex_param)



add_library(csapex_util SHARED
    src/utility/slim_signal.cpp
    src/utility/assert.cpp
    src/utility/exceptions.cpp
    src/utility/bash_parser.cpp
    src/utility/buffer.cpp
    src/utility/error_handling.cpp
    src/utility/stream_interceptor.cpp
    src/utility/stream_relay.cpp
    src/utility/thread.cpp
    src/utility/timable.cpp
    src/utility/timer.cpp
    src/utility/type.cpp
    src/utility/uuid.cpp
    src/utility/uuid_provider.cpp
    src/utility/yaml_node_builder.cpp
)

target_link_libraries(csapex_util
    ${Boost_LIBRARIES}
    ${YAML_LIBRARIES}
)

list(APPEND csapex_LIBRARIES csapex_util)


add_library(csapex SHARED
    src/utility/slim_signal_implementations.cpp

    src/model/node_handle.cpp
    src/model/node_worker.cpp

    src/core/csapex_core.cpp
    src/core/core_plugin.cpp
    src/core/bootstrap_plugin.cpp
    src/core/graphio.cpp
    src/core/settings.cpp
    src/core/exception_handler.cpp

    src/data/point.cpp

    src/factory/generic_node_factory.cpp
    src/factory/node_factory.cpp
    src/factory/message_factory.cpp

    src/manager/message_provider_manager.cpp

    src/model/connectable.cpp
    src/model/connection.cpp
    src/model/connection_type.cpp
    src/model/error_state.cpp
    src/model/fulcrum.cpp
    src/model/generic_state.cpp
    src/model/graph.cpp
    src/model/graph_facade.cpp
    src/model/memento.cpp
    src/model/multi_connection_type.cpp
    src/model/node_constructor.cpp
    src/model/node.cpp
    src/model/node_modifier.cpp
    src/model/node_runner.cpp
    src/model/node_state.cpp
    src/model/parameterizable.cpp
    src/model/tickable_node.cpp
    src/model/tag.cpp
    src/model/unique.cpp

    src/msg/apex_message_provider.cpp
    src/msg/dynamic_input.cpp
    src/msg/dynamic_output.cpp
    src/msg/input.cpp
    src/msg/input_transition.cpp
    src/msg/io.cpp
    src/msg/message.cpp
    src/msg/any_message.cpp
    src/msg/no_message.cpp
    src/msg/message_provider.cpp
    src/msg/output.cpp
    src/msg/output_transition.cpp
    src/msg/static_output.cpp
    src/msg/transition.cpp
    src/msg/bundled_connection.cpp
    src/msg/direct_connection.cpp

    src/serialization/yaml.cpp
    src/serialization/message_serializer.cpp
    src/serialization/serialization.cpp

    src/plugin/plugin_locator.cpp

    src/scheduling/executor.cpp
    src/scheduling/scheduler.cpp
    src/scheduling/task.cpp
    src/scheduling/task_generator.cpp
    src/scheduling/thread_group.cpp
    src/scheduling/thread_pool.cpp

    src/signal/signal.cpp
    src/signal/signal_connection.cpp
    src/signal/slot.cpp
    src/signal/trigger.cpp
)

target_link_libraries(csapex
    ${Boost_LIBRARIES}
    ${YAML_LIBRARIES}
    ${TinyXML_LIBRARIES}
    ${console_bridge_LIBRARIES}
    ${class_loader_LIBRARIES}
)

list(APPEND csapex_LIBRARIES csapex)


# Library for the app itself
add_library(csapex_qt SHARED
    src/command/command.cpp
    src/command/dispatcher.cpp
    src/command/meta.cpp
    src/command/command_factory.cpp
    src/command/add_connection.cpp
    src/command/add_msg_connection.cpp
    src/command/add_signal_connection.cpp
    src/command/add_fulcrum.cpp
    src/command/add_node.cpp
    src/command/create_thread.cpp
    src/command/delete_connection.cpp
    src/command/delete_msg_connection.cpp
    src/command/delete_signal_connection.cpp
    src/command/delete_connector.cpp
    src/command/delete_fulcrum.cpp
    src/command/delete_node.cpp
    src/command/disable_node.cpp
    src/command/flip_sides.cpp
    src/command/minimize.cpp
    src/command/modify_fulcrum.cpp
    src/command/move_box.cpp
    src/command/move_connection.cpp
    src/command/move_fulcrum.cpp
    src/command/switch_thread.cpp
    src/command/group_nodes.cpp
    src/command/paste_graph.cpp
    src/command/pass_out_connector.cpp

    src/view/csapex_window.cpp
    src/view/message_renderer.cpp
    src/view/gui_exception_handler.cpp

    src/manager/message_renderer_manager.cpp

    src/view/designer/designer.cpp
    src/view/designer/designer_scene.cpp
    src/view/designer/designer_styleable.cpp
    src/view/designer/graph_view.cpp
    src/view/designer/designerio.cpp
    src/view/designer/drag_io.cpp
    src/view/designer/drag_io_handler.cpp
    src/view/designer/fulcrum_handle.cpp
    src/view/designer/fulcrum_widget.cpp
    src/view/designer/widget_controller.cpp

    src/view/node/default_node_adapter.cpp
    src/view/node/node_adapter_builder.cpp
    src/view/node/node_adapter.cpp
    src/view/node/node_adapter_factory.cpp
    src/view/node/node_filter_proxy_model.cpp
    src/view/node/parameter_context_menu.cpp
    src/view/node/box.cpp
    src/view/node/node_statistics.cpp

    src/view/utility/node_list_generator.cpp
    src/view/utility/context_menu_handler.cpp
    src/view/utility/html_delegate.cpp
    src/view/utility/qsignal_bridges.cpp
    src/view/utility/qsignal_relay.cpp
    src/view/utility/QtCvImageConverter.cpp
    src/view/utility/qt_helper.cpp
    src/view/utility/qwrapper.cpp
    src/view/utility/widget_picker.cpp

    src/view/widgets/activity_legend.cpp
    src/view/widgets/activity_timeline.cpp
    src/view/widgets/box_dialog.cpp
    src/view/widgets/csapex_splashscreen.cpp
    src/view/widgets/doublespanslider.cpp
    src/view/widgets/minimap_widget.cpp
    src/view/widgets/port.cpp
    src/view/widgets/profiling_widget.cpp
    src/view/widgets/qdouble_slider.cpp
    src/view/widgets/qint_slider.cpp
    src/view/widgets/screenshot_dialog.cpp
    src/view/widgets/movable_graphics_proxy_widget.cpp
    src/view/widgets/message_preview_widget.cpp

    ${QT_RESOURCES}

    ${QT_UI}
)

target_link_libraries(csapex_qt
    ${csapex_LIBRARIES}

    qxt5

    Qt5::Core Qt5::Gui Qt5::Widgets Qt5::OpenGL
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${YAML_LIBRARIES}
    ${TinyXML_LIBRARIES}
    ${console_bridge_LIBRARIES}   
    ${class_loader_LIBRARIES}
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

target_link_libraries(csapex_node
    csapex_qt
    ${Boost_LIBRARIES})


add_custom_target(cfg ALL DEPENDS cfg/style.css)
add_custom_command(TARGET cfg POST_BUILD
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/setup.sh
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
)

#
# Enable IWYU
#
#if(${CMAKE_VERSION} VERSION_GREATER 3.2)
#  find_program(iwyu_path NAMES include-what-you-use iwyu)
#  if(NOT iwyu_path)
#    message(FATAL_ERROR "Could not find the program include-what-you-use")
#  endif()
#
#  message("Using include-what-you-use from ${iwyu_path}")
#
#  set_property(TARGET csapex PROPERTY CXX_INCLUDE_WHAT_YOU_USE ${iwyu_path})
#  set_property(TARGET csapex_qt PROPERTY CXX_INCLUDE_WHAT_YOU_USE ${iwyu_path})
#endif()


#
# TESTS
#
if(EXISTS ${CMAKE_CURRENT_LIST_DIR}/tests AND EXISTS ${CMAKE_CURRENT_LIST_DIR}/tests/CMakeLists.txt)
    add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/tests)
endif()

#
# INSTALL
#

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS csapex
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(TARGETS csapex_qt
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(TARGETS csapex_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#
# ANALYSIS TOOLS
#

get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)

set(directories "")
foreach(dir ${dirs})
  #message(STATUS "dir='${dir}'")
  set(directories "-I${dir} ${directories}")
endforeach()

add_custom_command(OUTPUT includes.txt
                   COMMAND eval "echo ${directories} >includes.txt")
add_custom_target(generate_includes ALL DEPENDS includes.txt)
