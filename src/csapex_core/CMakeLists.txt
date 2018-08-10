cmake_minimum_required(VERSION 2.8.11)

project(csapex_core)

## Add our own cmake subdirectory
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(cmake/csapex-extras.cmake)
include (GenerateExportHeader)

## Find catkin macros and libraries
find_package(catkin QUIET)

find_package(class_loader REQUIRED)
find_package(csapex_util REQUIRED)

find_package(Boost COMPONENTS program_options filesystem system regex  iostreams REQUIRED)

find_package(TinyXML REQUIRED)
find_package(YAML-CPP REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_BINARY_DIR})


## CATKIN
if(${catkin_FOUND})
    catkin_package(
       INCLUDE_DIRS
          include
          ${CMAKE_CURRENT_BINARY_DIR}
          ${YAML-CPP_INCLUDE_DIR}
       LIBRARIES
          ${PROJECT_NAME}
          csapex_param
          csapex_serialization
          csapex_command
          csapex_profiling
          ${YAML-CPP_LIBRARY}
       DEPENDS
          Boost
       CATKIN_DEPENDS
          csapex_util
          class_loader

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
find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif(CCACHE_FOUND)

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
file(APPEND ${INFO} "static const std::string MAINTAINER { \"${csapex_core_MAINTAINER}\" };\n")
file(APPEND ${INFO} "static const std::string VERSION { \"${csapex_core_VERSION}\" };\n")
file(APPEND ${INFO} "static const std::string CSAPEX_DEFAULT_BOOT_PLUGIN_DIR { \"${CSAPEX_BOOT_PLUGIN_DIR}\" };\n")
file(APPEND ${INFO} "static const std::string CSAPEX_VERSION { \"${csapex_core_VERSION}\" };\n")

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

include_directories(include
    ${YAML-CPP_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${csapex_util_INCLUDE_DIRS}
    ${class_loader_INCLUDE_DIRS}
    ${console_bridge_INCLUDE_DIRS}
)


#
# BUILD THE LIBRARIES
#

set(csapex_LIBRARIES)

file(GLOB_RECURSE csapex_serialization_HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/csapex/serializationity/*.h*)

add_library(csapex_serialization SHARED

    src/serialization/serializable.cpp
    src/serialization/yaml.cpp
    src/serialization/message_serializer.cpp
    src/serialization/node_serializer.cpp
    src/serialization/snippet.cpp
    src/serialization/packet_serializer.cpp
    src/serialization/serialization_buffer.cpp

    src/serialization/io/std_io.cpp
    src/serialization/io/boost_io.cpp
    src/serialization/io/csapex_io.cpp

    ${csapex_serialization_HEADERS}
)
generate_export_header( csapex_serialization
             BASE_NAME csapex_serialization
             EXPORT_MACRO_NAME CSAPEX_serializationS_EXPORT
             EXPORT_FILE_NAME csapex_serialization_export.h
             STATIC_DEFINE csapex_serialization_BUILT_AS_STATIC
                         )
target_link_libraries(csapex_serialization
    ${csapex_util_LIBRARIES}
    ${Boost_LIBRARIES}
    ${YAML-CPP_LIBRARY}
)

list(APPEND csapex_LIBRARIES csapex_serialization)


file(GLOB_RECURSE csapex_param_HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/csapex/param/*.h*)

add_library(csapex_param SHARED
    src/param/io.cpp
    src/param/parameter.cpp
    src/param/parameter_builder.cpp
    src/param/parameter_provider.cpp
    src/param/parameter_description.cpp

    src/param/range_parameter.cpp
    src/param/interval_parameter.cpp
    src/param/set_parameter.cpp
    src/param/bitset_parameter.cpp
    src/param/value_parameter.cpp
    src/param/path_parameter.cpp
    src/param/trigger_parameter.cpp
    src/param/color_parameter.cpp
    src/param/null_parameter.cpp
    src/param/string_list_parameter.cpp

    src/param/output_text_parameter.cpp
    src/param/output_progress_parameter.cpp

    src/param/parameter_factory.cpp

    src/serialization/parameter_serializer.cpp

    ${csapex_param_HEADERS}
)
generate_export_header( csapex_param
             BASE_NAME csapex_param
             EXPORT_MACRO_NAME CSAPEX_PARAM_EXPORT
             EXPORT_FILE_NAME csapex_param_export.h
             STATIC_DEFINE csapex_param_BUILT_AS_STATIC
			 )
target_link_libraries(csapex_param
    ${csapex_util_LIBRARIES}
    csapex_serialization
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
    src/profiling/profiler_impl.cpp
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
    ${csapex_util_LIBRARIES}
    ${Boost_LIBRARIES}
    ${YAML-CPP_LIBRARY}
)

list(APPEND csapex_LIBRARIES csapex_profiling)



file(GLOB_RECURSE csapex_HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS 
        include/csapex/model/*.h*
        include/csapex/core/*.h*
        include/csapex/data/*.h*
        include/csapex/factory/*.h*
        include/csapex/msg/*.h*
        include/csapex/signal/*.h*
        include/csapex/serialization/*.h*
        include/csapex/plugin/*.h*
        include/csapex/scheduling/*.h*
        include/csapex/io/*.h*
)

add_library(${PROJECT_NAME} SHARED
    src/model/io.cpp
    src/model/node_facade.cpp
    src/model/node_facade_impl.cpp

    src/model/node_handle.cpp
    src/model/node_worker.cpp
    src/model/direct_node_worker.cpp
    src/model/subprocess_node_worker.cpp

    src/core/csapex_core.cpp
    src/core/core_plugin.cpp
    src/core/bootstrap.cpp
    src/core/bootstrap_plugin.cpp
    src/core/graphio.cpp
    src/core/exception_handler.cpp

    src/core/settings.cpp
    src/core/settings/settings_impl.cpp

    src/command/command.cpp
    src/command/dispatcher.cpp

    src/data/point.cpp

    src/factory/generic_node_factory.cpp
    src/factory/node_factory.cpp
    src/factory/node_factory_impl.cpp
    src/factory/message_factory.cpp
    src/factory/snippet_factory.cpp

    src/manager/message_provider_manager.cpp
    src/manager/message_renderer_manager.cpp

    src/model/graph/vertex.cpp
    src/model/graph/edge.cpp

    src/model/connector.cpp
    src/model/connectable.cpp
    src/model/connectable_owner.cpp
    src/model/connector_type.cpp
    src/model/connector_description.cpp
    src/model/connectable_vector.cpp
    src/model/connection.cpp
    src/model/connection_description.cpp
    src/model/token.cpp
    src/model/token_data.cpp
    src/model/error_state.cpp
    src/model/fulcrum.cpp
    src/model/generic_state.cpp
    src/model/graph.cpp
    src/model/graph/graph_impl.cpp
    src/model/subgraph_node.cpp
    src/model/graph_facade.cpp
    src/model/graph_facade_impl.cpp
    src/model/notification.cpp
    src/model/notifier.cpp
    src/model/multi_connection_type.cpp
    src/model/node_constructor.cpp
    src/model/node.cpp
    src/model/node_modifier.cpp
    src/model/node_runner.cpp
    src/model/node_state.cpp
    src/model/node_characteristics.cpp
    src/model/observer.cpp
    src/model/parameterizable.cpp
    src/model/throttled_node.cpp
    src/model/tag.cpp
    src/model/unique.cpp
    src/model/variadic_io.cpp

    src/nodes/sticky_note.cpp

    src/msg/token_traits.cpp
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
    src/msg/message_renderer.cpp
    src/msg/message_allocator.cpp

    src/plugin/plugin_locator.cpp

    src/scheduling/executor.cpp
    src/scheduling/scheduler.cpp
    src/scheduling/task.cpp
    src/scheduling/task_generator.cpp
    src/scheduling/thread_group.cpp
    src/scheduling/thread_pool.cpp
    src/scheduling/timed_queue.cpp

    src/signal/slot.cpp
    src/signal/event.cpp

    src/io/server.cpp

    ${csapex_HEADERS}
)

generate_export_header( ${PROJECT_NAME}
             BASE_NAME ${PROJECT_NAME}
             EXPORT_MACRO_NAME CSAPEX_CORE_EXPORT
             EXPORT_FILE_NAME ${PROJECT_NAME}_export.h
             STATIC_DEFINE ${PROJECT_NAME}_BUILT_AS_STATIC
			 )
target_link_libraries(${PROJECT_NAME}
    PUBLIC
    ${csapex_util_LIBRARIES}
    csapex_profiling
    csapex_param
    csapex_serialization
    ${Boost_LIBRARIES}
    ${YAML-CPP_LIBRARY}

    PRIVATE
    ${TinyXML_LIBRARIES}
    ${class_loader_LIBRARIES}
    ${console_bridge_LIBRARIES} 
    ${Poco_LIBRARIES}
)

list(APPEND csapex_LIBRARIES ${PROJECT_NAME})




file(GLOB_RECURSE csapex_command_HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/csapex/command/*.h*)

add_library(csapex_command SHARED
    src/command/command_factory.cpp
    src/command/command_serializer.cpp

    src/command/meta.cpp

    src/command/add_connection.cpp
    src/command/add_fulcrum.cpp
    src/command/add_node.cpp
    src/command/create_thread.cpp
    src/command/clear_graph.cpp
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
    src/command/modify_thread.cpp
    src/command/delete_thread.cpp
    src/command/group_base.cpp
    src/command/group_nodes.cpp
    src/command/ungroup_nodes.cpp
    src/command/paste_graph.cpp
    src/command/add_variadic_connector.cpp
    src/command/add_variadic_connector_and_connect.cpp
    src/command/set_color.cpp
    src/command/update_parameter.cpp
    src/command/set_execution_mode.cpp
    src/command/set_isolated_execution.cpp
    src/command/set_logger_level.cpp
    src/command/set_max_execution_frequency.cpp

    src/command/quit.cpp

    ${csapex_command_HEADERS}
)

generate_export_header( csapex_command
             BASE_NAME csapex_command
             EXPORT_MACRO_NAME CSAPEX_COMMAND_EXPORT
             EXPORT_FILE_NAME csapex_command_export.h
             STATIC_DEFINE csapex_command_BUILT_AS_STATIC
)
target_link_libraries(csapex_command
    ${csapex_util_LIBRARIES}
    csapex_param
    csapex_serialization
    ${PROJECT_NAME}

    ${Boost_LIBRARIES}
    ${csapex_util_LIBRARIES}
    ${YAML-CPP_LIBRARY}
    ${TinyXML_LIBRARIES}
    ${class_loader_LIBRARIES}
)

list(APPEND csapex_LIBRARIES csapex_command)


##
## Enable IWYU
##

if (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    if(${CMAKE_VERSION} VERSION_GREATER 3.2)
        find_program(iwyu_path NAMES include-what-you-use iwyu)
        if(iwyu_path)
            message("Using include-what-you-use from ${iwyu_path}")
            add_definitions(--system-header-prefix=/usr/include)

            foreach(lib ${csapex_LIBRARIES})
                set_property(TARGET ${lib} PROPERTY CXX_INCLUDE_WHAT_YOU_USE ${iwyu_path})
            endforeach()
        endif()
    endif()
endif()


#
# CFG
#
add_custom_target(${PROJECT_NAME}_cfg ALL DEPENDS cfg/intro.html)
add_custom_command(TARGET ${PROJECT_NAME}_cfg POST_BUILD
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/setup.sh
        ${CATKIN_DEVEL_PREFIX}
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
)


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
                #LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
                DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
endif()
		
if(NOT ${catkin_FOUND})
	install(FILES cmake/csapex-config.cmake
			DESTINATION ${CSAPEX_MODULE_DESTINATION})
	install(FILES cmake/Findcsapex_plugin.cmake
			DESTINATION ${CSAPEX_MODULE_DESTINATION})
	install(FILES cmake/csapex-extras.cmake
			DESTINATION ${CSAPEX_MODULE_DESTINATION})
endif()
