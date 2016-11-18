cmake_minimum_required(VERSION 2.8.11)

project(csapex)

## Add our own cmake subdirectory
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(cmake/csapex-extras.cmake)
include (GenerateExportHeader)

## Enable Profiling
#add_definitions(-ftime-report)

## Find catkin macros and libraries
find_package(catkin QUIET)

find_package(class_loader REQUIRED)

find_package(Qt5 COMPONENTS Core Gui Widgets OpenGL REQUIRED)
find_package(Boost COMPONENTS program_options filesystem system regex serialization thread iostreams REQUIRED)

find_package(OpenGL REQUIRED)
find_package(TinyXML REQUIRED)
find_package(YAML-CPP REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

qt5_add_resources(QT_RESOURCES res/csapex_resources.qrc)
qt5_wrap_ui(QT_UI
    ui/box.ui
    ui/csapex_window.ui
    ui/designer.ui)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_BINARY_DIR})


## CATKIN
if(${catkin_FOUND})
    catkin_package(
       INCLUDE_DIRS include
       # CATKIN_DEPENDS
       DEPENDS Boost
       CFG_EXTRAS csapex-extras.cmake
    )
else()
    message("building without catkin")

	set(Poco_DIR cmake)
	find_package(Poco REQUIRED COMPONENTS Foundation)
	find_package(console_bridge REQUIRED)
	set(INSTALL_DIR ${CMAKE_INSTALL_PREFIX})
	set(CATKIN_PACKAGE_INCLUDE_DESTINATION ${INSTALL_DIR}/include)
	set(CATKIN_PACKAGE_LIB_DESTINATION ${INSTALL_DIR}/lib)
	set(CATKIN_GLOBAL_BIN_DESTINATION ${INSTALL_DIR}/bin)
	set(CATKIN_PACKAGE_SHARE_DESTINATION ${INSTALL_DIR}/share)
	set(CSAPEX_MODULE_DESTINATION ${INSTALL_DIR}/CMake)

	set(CSAPEX_BOOT_PLUGIN_DIR ${CATKIN_GLOBAL_BIN_DESTINATION}/csapex_plugins)

	
	FILE(READ ${CMAKE_CURRENT_LIST_DIR}/package.xml package)

	string(REGEX MATCH "^.*<version>([^<]*)</version>.*$" match ${package})	
	if(match)
		set(csapex_VERSION ${CMAKE_MATCH_1})
	endif()

	string(REGEX MATCH "^.*<maintainer.*email=\"[^\"]*\">([^<]*)</maintainer>.*$" match ${package})
	if(match)
		set(csapex_MAINTAINER ${CMAKE_MATCH_1})
	endif()
	
	if(WIN32)	
		# disable min and max macros
		add_definitions(/D NOMINMAX)
	endif()
endif()


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

if(WIN32) 
	message(STATUS ${INFO} " -> " ${INFO_H})
	file(RENAME ${INFO} ${INFO_H})
else()
	execute_process(COMMAND bash -c "if [[ ! -f info.h  ]] || [[ `diff info.h.tmp info.h` ]]; then mv info.h.tmp info.h; else rm info.h.tmp; fi"
				    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/include/csapex/)
endif()	

###########
## Build ##
###########

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/external)

include_directories(include

    external/qxt5/include
	${YAML-CPP_INCLUDE_DIR}
	${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${OPENGL_INCLUDE_DIR}
    ${Qt5Core_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS}
	${console_bridge_INCLUDE_DIRS} 
)

if (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_definitions(--system-header-prefix=/usr/include)
endif()

#
# BUILD THE LIBRARIES
#

set(csapex_LIBRARIES)


file(GLOB_RECURSE csapex_util_HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/csapex/utility/*.h*)

add_library(csapex_util SHARED
    src/utility/slim_signal.cpp
    src/utility/assert.cpp
    src/utility/exceptions.cpp
    src/utility/bash_parser.cpp
    src/utility/error_handling.cpp
    src/utility/stream_interceptor.cpp
    src/utility/stream_relay.cpp
    src/utility/singleton.cpp
    src/utility/thread.cpp
    src/utility/rate.cpp
    src/utility/type.cpp
    src/utility/uuid.cpp
    src/utility/uuid_provider.cpp
    src/utility/yaml_node_builder.cpp
    src/utility/slim_signal_implementations.cpp

    ${csapex_util_HEADERS}
)
generate_export_header( csapex_util
             BASE_NAME csapex_util
             EXPORT_MACRO_NAME CSAPEX_UTILS_EXPORT
             EXPORT_FILE_NAME csapex_util_export.h
             STATIC_DEFINE csapex_util_BUILT_AS_STATIC
			 )
target_link_libraries(csapex_util
    ${Boost_LIBRARIES}
    ${YAML-CPP_LIBRARY}
)

list(APPEND csapex_LIBRARIES csapex_util)



file(GLOB_RECURSE csapex_param_HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/csapex/param/*.h*)

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

    src/param/output_text_parameter.cpp
    src/param/output_progress_parameter.cpp

    src/param/parameter_factory.cpp
    src/param/parameter_map.cpp
    src/param/parameter_provider.cpp
	
    ${csapex_param_HEADERS}
)
generate_export_header( csapex_param
             BASE_NAME csapex_param
             EXPORT_MACRO_NAME CSAPEX_PARAM_EXPORT
             EXPORT_FILE_NAME csapex_param_export.h
             STATIC_DEFINE csapex_param_BUILT_AS_STATIC
			 )
target_link_libraries(csapex_param
    csapex_util
    ${Boost_LIBRARIES}
    ${YAML-CPP_LIBRARY}
)

list(APPEND csapex_LIBRARIES csapex_param)




file(GLOB_RECURSE csapex_profiling_HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/csapex/profiling/*.h*)

add_library(csapex_profiling SHARED
    src/profiling/timer.cpp
    src/profiling/interval.cpp
    src/profiling/interlude.cpp
    src/profiling/profile.cpp
    src/profiling/timer.cpp
    src/profiling/profiler.cpp
    src/profiling/timable.cpp
    src/profiling/profilable.cpp

	${csapex_profiling_HEADERS}
)
generate_export_header( csapex_profiling
             BASE_NAME csapex_profiling
             EXPORT_MACRO_NAME CSAPEX_PROFILING_EXPORT
             EXPORT_FILE_NAME csapex_profiling_export.h
             STATIC_DEFINE csapex_profiling_BUILT_AS_STATIC
			 )

target_link_libraries(csapex_profiling
    csapex_util
    ${Boost_LIBRARIES}
    ${YAML-CPP_LIBRARY}
)

list(APPEND csapex_LIBRARIES csapex_profiling)



file(GLOB_RECURSE csapex_HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS 
        include/csapex/model/*.h*
        include/csapex/data/*.h*
        include/csapex/factory/*.h*
        include/csapex/msg/*.h*
        include/csapex/signal/*.h*
        include/csapex/serialization/*.h*
        include/csapex/plugin/*.h*
        include/csapex/scheduling/*.h*
)

add_library(csapex SHARED
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
    src/factory/snippet_factory.cpp

    src/manager/message_provider_manager.cpp

    src/model/graph/vertex.cpp
    src/model/graph/edge.cpp

    src/model/connectable.cpp
    src/model/connectable_owner.cpp
    src/model/connector_type.cpp
    src/model/connection.cpp
    src/model/connection_information.cpp
    src/model/token.cpp
    src/model/token_data.cpp
    src/model/error_state.cpp
    src/model/fulcrum.cpp
    src/model/generic_state.cpp
    src/model/graph.cpp
    src/model/subgraph_node.cpp
    src/model/graph_facade.cpp
    src/model/memento.cpp
    src/model/multi_connection_type.cpp
    src/model/node_constructor.cpp
    src/model/node.cpp
    src/model/node_modifier.cpp
    src/model/node_runner.cpp
    src/model/node_state.cpp
    src/model/node_characteristics.cpp
    src/model/parameterizable.cpp
    src/model/generator_node.cpp
    src/model/tickable_node.cpp
    src/model/tag.cpp
    src/model/unique.cpp
    src/model/variadic_io.cpp

    src/nodes/note.cpp

    src/msg/apex_message_provider.cpp
    src/msg/input.cpp
    src/msg/input_transition.cpp
    src/msg/io.cpp
    src/msg/message.cpp
    src/msg/any_message.cpp
    src/msg/marker_message.cpp
    src/msg/no_message.cpp
    src/msg/end_of_sequence_message.cpp
    src/msg/end_of_program_message.cpp
    src/msg/message_provider.cpp
    src/msg/output.cpp
    src/msg/output_transition.cpp
    src/msg/static_output.cpp
    src/msg/transition.cpp
    src/msg/direct_connection.cpp
    src/msg/generic_vector_message.cpp

    src/serialization/yaml.cpp
    src/serialization/message_serializer.cpp
    src/serialization/serialization.cpp
    src/serialization/snippet.cpp

    src/plugin/plugin_locator.cpp

    src/scheduling/executor.cpp
    src/scheduling/scheduler.cpp
    src/scheduling/task.cpp
    src/scheduling/task_generator.cpp
    src/scheduling/thread_group.cpp
    src/scheduling/thread_pool.cpp

    src/signal/slot.cpp
    src/signal/event.cpp

	${csapex_HEADERS}
)

generate_export_header( csapex
             BASE_NAME csapex
             EXPORT_MACRO_NAME CSAPEX_EXPORT
             EXPORT_FILE_NAME csapex_export.h
             STATIC_DEFINE csapex_BUILT_AS_STATIC
			 )
target_link_libraries(csapex
    csapex_util
    csapex_profiling
    csapex_param
    ${Boost_LIBRARIES}
    ${YAML-CPP_LIBRARY}
    ${TinyXML_LIBRARIES}
    ${class_loader_LIBRARIES}
    ${console_bridge_LIBRARIES} 
    ${Poco_LIBRARIES}
)

list(APPEND csapex_LIBRARIES csapex)




file(GLOB_RECURSE csapex_command_HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/csapex/command/*.h*)

add_library(csapex_command SHARED
    src/command/command.cpp
    src/command/dispatcher.cpp
    src/command/meta.cpp
    src/command/playback_command.cpp
    src/command/command_factory.cpp
    src/command/add_connection.cpp
    src/command/add_fulcrum.cpp
    src/command/add_node.cpp
    src/command/create_thread.cpp
    src/command/delete_connection.cpp
    src/command/delete_connector.cpp
    src/command/delete_fulcrum.cpp
    src/command/delete_node.cpp
    src/command/disable_node.cpp
    src/command/flip_sides.cpp
    src/command/minimize.cpp
    src/command/mute_node.cpp
    src/command/modify_fulcrum.cpp
    src/command/modify_connection.cpp
    src/command/move_box.cpp
    src/command/move_fulcrum.cpp
    src/command/rename_node.cpp
    src/command/rename_connector.cpp
    src/command/switch_thread.cpp
    src/command/group_base.cpp
    src/command/group_nodes.cpp
    src/command/ungroup_nodes.cpp
    src/command/paste_graph.cpp
    src/command/add_variadic_connector.cpp
    src/command/set_color.cpp
    src/command/update_parameter.cpp
    src/command/set_execution_mode.cpp
    src/command/set_logger_level.cpp

	${csapex_command_HEADERS}
)

generate_export_header( csapex_command
             BASE_NAME csapex_command
             EXPORT_MACRO_NAME CSAPEX_COMMAND_EXPORT
             EXPORT_FILE_NAME csapex_command_export.h
             STATIC_DEFINE csapex_command_BUILT_AS_STATIC
			 )
target_link_libraries(csapex_command
    csapex_util
	csapex_param
	csapex
	
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${YAML-CPP_LIBRARY}
    ${TinyXML_LIBRARIES}
    ${class_loader_LIBRARIES}
)

list(APPEND csapex_LIBRARIES csapex_command)



file(GLOB_RECURSE csapex_qt_HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/csapex/view/*.h*)

if(NOT WIN32)
    # for visual studio, we want the headers in the library
    # however, with ubuntu that doesn't compile due to auto moc'ing
#    add_custom_target(csapex_qt_headers SOURCES ${csapex_qt_HEADERS})
    set(csapex_qt_HEADERS)
endif()

# Library for the app itself
add_library(csapex_qt SHARED

    src/view/csapex_view_core.cpp
    src/view/csapex_window.cpp
    src/view/message_renderer.cpp
    src/view/gui_exception_handler.cpp

    src/manager/message_renderer_manager.cpp

    src/view/designer/designer.cpp
    src/view/designer/designer_options.cpp
    src/view/designer/designer_scene.cpp
    src/view/designer/designer_styleable.cpp
    src/view/designer/graph_view.cpp
    src/view/designer/graph_view_context_menu.cpp
    src/view/designer/designerio.cpp
    src/view/designer/drag_io.cpp
    src/view/designer/drag_io_handler.cpp
    src/view/designer/fulcrum_handle.cpp
    src/view/designer/fulcrum_widget.cpp
    src/view/designer/tutorial_tree_model.cpp

    src/view/node/default_node_adapter.cpp
    src/view/node/node_adapter_builder.cpp
    src/view/node/node_adapter.cpp
    src/view/node/node_adapter_factory.cpp
    src/view/node/node_filter_proxy_model.cpp
    src/view/node/parameter_context_menu.cpp
    src/view/node/box.cpp
    src/view/node/note_box.cpp
    src/view/node/node_statistics.cpp

    src/view/param/param_adapter.cpp
    src/view/param/range_param_adapter.cpp
    src/view/param/value_param_adapter.cpp


    src/view/param/range_param_adapter.cpp
    src/view/param/interval_param_adapter.cpp
    src/view/param/set_param_adapter.cpp
    src/view/param/bitset_param_adapter.cpp
    src/view/param/value_param_adapter.cpp
    src/view/param/path_param_adapter.cpp
    #src/view/param/trigger_param_adapter.cpp
    src/view/param/color_param_adapter.cpp
    src/view/param/angle_param_adapter.cpp
    #src/view/param/null_param_adapter.cpp
    #src/view/param/string_list_param_adapter.cpp

    src/view/param/output_progress_param_adapter.cpp
    src/view/param/output_text_param_adapter.cpp

    src/view/utility/node_list_generator.cpp
    src/view/utility/snippet_list_generator.cpp
    src/view/utility/context_menu_handler.cpp
    src/view/utility/html_delegate.cpp
    src/view/utility/qsignal_bridges.cpp
    src/view/utility/qsignal_relay.cpp
    src/view/utility/qt_helper.cpp
    src/view/utility/qwrapper.cpp
    src/view/utility/widget_picker.cpp
    src/view/utility/clipboard.cpp

    src/view/widgets/activity_legend.cpp
    src/view/widgets/activity_timeline.cpp
    src/view/widgets/activity_timeline_item.cpp
    src/view/widgets/completed_line_edit.cpp
    src/view/widgets/box_dialog.cpp
    src/view/widgets/search_dialog.cpp
    src/view/widgets/rewiring_dialog.cpp
    src/view/widgets/csapex_splashscreen.cpp
    src/view/widgets/doublespanslider.cpp
    src/view/widgets/minimap_widget.cpp
    src/view/widgets/port.cpp
    src/view/widgets/meta_port.cpp
    src/view/widgets/port_panel.cpp
    src/view/widgets/profiling_widget.cpp
    src/view/widgets/qdouble_slider.cpp
    src/view/widgets/qint_slider.cpp
    src/view/widgets/screenshot_dialog.cpp
    src/view/widgets/movable_graphics_proxy_widget.cpp
    src/view/widgets/message_preview_widget.cpp
    src/view/widgets/notification_widget.cpp

    ${csapex_qt_HEADERS}

    ${QT_RESOURCES}

    ${QT_UI}
)

generate_export_header( csapex_qt
             BASE_NAME csapex_qt
             EXPORT_MACRO_NAME CSAPEX_QT_EXPORT_CMAKE
             EXPORT_FILE_NAME csapex_qt_export.h
             STATIC_DEFINE csapex_qt_BUILT_AS_STATIC
			 )
target_link_libraries(csapex_qt
    ${csapex_LIBRARIES}

    qxt5

    Qt5::Core Qt5::Gui Qt5::Widgets Qt5::OpenGL
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${YAML-CPP_LIBRARY}
    ${TinyXML_LIBRARIES}
    ${class_loader_LIBRARIES}
)
list(APPEND csapex_LIBRARIES csapex_qt)

set_target_properties(csapex_qt PROPERTIES AUTOMOC TRUE)


#
# BUILD THE APPLICATION
#

if(WIN32)
	add_executable(csapex_node
		src/csapex.cpp
		res/csapex.rc
	)
else()
        add_executable(csapex_node
		src/csapex.cpp)
endif()

set_target_properties(csapex_node PROPERTIES AUTOMOC TRUE)
target_link_libraries(csapex_node
    csapex_qt
    ${Boost_LIBRARIES})


add_custom_target(cfg ALL DEPENDS cfg/style.css)
add_custom_command(TARGET cfg POST_BUILD
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/setup.sh
        ${CATKIN_DEVEL_PREFIX}
        ${PROJECT_NAME}/csapex_node
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
)

##
## Enable IWYU
##
#if(${CMAKE_VERSION} VERSION_GREATER 3.2)
#  find_program(iwyu_path NAMES include-what-you-use iwyu)
#  if(NOT iwyu_path)
#    message(FATAL_ERROR "Could not find the program include-what-you-use")
#  endif()
#
#  message("Using include-what-you-use from ${iwyu_path}")
#
#  set_property(TARGET csapex_util PROPERTY CXX_INCLUDE_WHAT_YOU_USE ${iwyu_path})
#  set_property(TARGET csapex_param PROPERTY CXX_INCLUDE_WHAT_YOU_USE ${iwyu_path})
#  set_property(TARGET csapex_profiling PROPERTY CXX_INCLUDE_WHAT_YOU_USE ${iwyu_path})
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
foreach(lib ${csapex_LIBRARIES})
    if(WIN32)
        file(COPY ${CMAKE_CURRENT_BINARY_DIR}/${lib}_export.h
             DESTINATION ${CMAKE_CURRENT_LIST_DIR}/include/${PROJECT_NAME}/)
		install(FILES ${PROJECT_BINARY_DIR}/${lib}_export.h
				DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/${PROJECT_NAME}/)
    else()
        file(COPY ${CMAKE_CURRENT_BINARY_DIR}/${lib}_export.h
             DESTINATION ${CMAKE_CURRENT_LIST_DIR}/include/${PROJECT_NAME}/)
		install(FILES ${PROJECT_BINARY_DIR}/${lib}_export.h
				DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
    endif()

endforeach()

if(WIN32)
	install(DIRECTORY include/${PROJECT_NAME}/
			DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/${PROJECT_NAME}/)
else()
	install(DIRECTORY include/${PROJECT_NAME}/
			LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
			DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
endif()

install(TARGETS csapex
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(TARGETS csapex_param
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
		
install(TARGETS csapex_command
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
		
install(TARGETS csapex_profiling
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(TARGETS csapex_util
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(TARGETS csapex_qt
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(TARGETS csapex_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
		
		
if(NOT ${catkin_FOUND})
	install(FILES cmake/csapex-config.cmake
			DESTINATION ${CSAPEX_MODULE_DESTINATION})
	install(FILES cmake/Findcsapex_plugin.cmake
			DESTINATION ${CSAPEX_MODULE_DESTINATION})
	install(FILES cmake/csapex-extras.cmake
			DESTINATION ${CSAPEX_MODULE_DESTINATION})
endif()
#
# ANALYSIS TOOLS
#

if(WIN32)
else()
	get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)

	set(directories "")
	foreach(dir ${dirs})
	  #message(STATUS "dir='${dir}'")
	  set(directories "-I${dir} ${directories}")
	endforeach()

	add_custom_command(OUTPUT includes.txt
					   COMMAND eval "echo ${directories} >includes.txt")
	add_custom_target(generate_includes ALL DEPENDS includes.txt)
endif()
