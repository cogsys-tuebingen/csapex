cmake_minimum_required(VERSION 3.8.2)

project(csapex_remote CXX)

find_package(catkin REQUIRED COMPONENTS csapex_core)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# make sure the devel include exists, there we will generate the export headers
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

catkin_package(
   INCLUDE_DIRS
      include
      ${CATKIN_DEVEL_PREFIX}/include
   LIBRARIES
      csapex_remote
   CATKIN_DEPENDS
      csapex_core
)

include (GenerateExportHeader)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(include)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

include_directories(${catkin_INCLUDE_DIRS})

#
# BUILD
#


file(GLOB_RECURSE ${PROJECT_NAME}_HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS
    include/csapex/core/*.h*
    include/csapex/io/*.h*
)

add_library(${PROJECT_NAME} SHARED
    src/command/dispatcher_proxy.cpp

    src/core/settings/settings_proxy.cpp

    src/factory/node_factory_proxy.cpp

    src/io/broadcast_message.cpp
    src/io/channel.cpp
    src/io/connector_server.cpp
    src/io/feedback.cpp
    src/io/graph_server.cpp
    src/io/node_server.cpp
    src/io/note.cpp
    src/io/protocol/add_parameter.cpp
    src/io/protocol/command_broadcasts.cpp
    src/io/protocol/command_requests.cpp
    src/io/protocol/connector_notes.cpp
    src/io/protocol/connector_requests.cpp
    src/io/protocol/core_notes.cpp
    src/io/protocol/core_requests.cpp
    src/io/protocol/graph_broadcasts.cpp
    src/io/protocol/graph_facade_notes.cpp
    src/io/protocol/graph_facade_requests.cpp
    src/io/protocol/graph_notes.cpp
    src/io/protocol/graph_requests.cpp
    src/io/protocol/node_broadcasts.cpp
    src/io/protocol/node_notes.cpp
    src/io/protocol/node_requests.cpp
    src/io/protocol/notification_message.cpp
    src/io/protocol/parameter_changed.cpp
    src/io/protocol/profiler_note.cpp
    src/io/protocol/profiler_requests.cpp
    src/io/protocol/request_nodes.cpp
    src/io/protocol/request_parameter.cpp
    src/io/protocol/tick_message.cpp
    src/io/proxy.cpp
    src/io/raw_message.cpp
    src/io/request.cpp
    src/io/response.cpp
    src/io/session_client.cpp
    src/io/session.cpp
    src/io/tcp_server.cpp

    src/model/connector_proxy.cpp
    src/model/graph_facade_proxy.cpp
    src/model/graph/graph_proxy.cpp
    src/model/node_facade_proxy.cpp

    src/profiling/profiler_proxy.cpp

    src/serialization/broadcast_message_serializer.cpp
    src/serialization/feedback_serializer.cpp
    src/serialization/note_serializer.cpp
    src/serialization/raw_message_serializer.cpp
    src/serialization/request_serializer.cpp

    ${${PROJECT_NAME}_HEADERS}
)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    Threads::Threads
)

generate_export_header( ${PROJECT_NAME}
             BASE_NAME ${PROJECT_NAME}
             EXPORT_MACRO_NAME CSAPEX_REMOTE_EXPORT
             EXPORT_FILE_NAME ${PROJECT_NAME}/export.h
             STATIC_DEFINE ${PROJECT_NAME}_BUILT_AS_STATIC
)

#
# INSTALL
#


set(${PROJECT_NAME}_EXPORT_DIR ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION})
set(${PROJECT_NAME}_EXPORT_FILE ${${PROJECT_NAME}_EXPORT_DIR}/export.h)

file(COPY ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}/export.h
    DESTINATION ${${PROJECT_NAME}_EXPORT_DIR})
install(FILES ${${PROJECT_NAME}_EXPORT_FILE}
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY include/csapex/
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/csapex)

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
