cmake_minimum_required(VERSION 3.8.2)

project(csapex_util CXX)

find_package(catkin REQUIRED)
find_package(Boost COMPONENTS regex system REQUIRED)

enable_testing()

include (GenerateExportHeader)

## Enforce that we use C++17
if(CMAKE_VERSION VERSION_LESS "3.1")
  message(FATAL_ERROR "CMake Version < 3.1 is no longer supported.")
else()
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)

# make sure the devel include exists, there we will generate the export headers
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

catkin_package(
   INCLUDE_DIRS
      include
      ${CATKIN_DEVEL_PREFIX}/include
   LIBRARIES
      ${PROJECT_NAME}
      rt
   DEPENDS
      Boost
)

#
# LIB
#
add_library(${PROJECT_NAME} SHARED
    src/slim_signal.cpp
    src/assert.cpp
    src/exceptions.cpp
    src/bash_parser.cpp
    src/error_handling.cpp
    src/stream_interceptor.cpp
    src/stream_relay.cpp
    src/singleton.cpp
    src/thread.cpp
    src/rate.cpp
    src/type.cpp
    src/uuid.cpp
    src/uuid_provider.cpp
    src/yaml_node_builder.cpp
    src/slim_signal_implementations.cpp
    src/ticker.cpp
    src/cpu_affinity.cpp
    src/subprocess_channel.cpp
    src/subprocess.cpp
    src/semantic_version.cpp

    ${csapex_util_HEADERS}
)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${CURRENT_BINARY_DIR})

# link rt library for shared memory
target_link_libraries(${PROJECT_NAME} -lrt)

# Define headers for this library. PUBLIC headers are used for
# compiling the library, and will be added to consumers' build
# paths.
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)
target_link_libraries(${PROJECT_NAME}
    ${Boost_LIBRARIES}
    ${YAML-CPP_LIBRARY}
)


generate_export_header( ${PROJECT_NAME}
             BASE_NAME ${PROJECT_NAME}
             EXPORT_MACRO_NAME CSAPEX_UTILS_EXPORT
             EXPORT_FILE_NAME ${PROJECT_NAME}/export.h
             STATIC_DEFINE ${PROJECT_NAME}_BUILT_AS_STATIC)

#
# INSTALL
#

set(${PROJECT_NAME}_EXPORT_DIR ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION})
set(${PROJECT_NAME}_EXPORT_FILE ${${PROJECT_NAME}_EXPORT_DIR}/export.h)

file(COPY ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}/export.h
    DESTINATION ${${PROJECT_NAME}_EXPORT_DIR})

install(FILES ${${PROJECT_NAME}_EXPORT_FILE}
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS csapex_util
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/csapex/utility/
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/csapex/utility)

export(TARGETS csapex_util FILE CsApexUtil.cmake)

#
# TEST
#

include( CTest )

include_directories(${GTEST_INCLUDE_DIR} ../include)

add_executable(${PROJECT_NAME}_tests
    tests/slim_signals_test.cpp
    tests/uuid_test.cpp
    tests/shared_memory_test.cpp
    tests/type_test.cpp
)

add_test(NAME ${PROJECT_NAME}_test COMMAND ${PROJECT_NAME}_tests)
set_tests_properties(${PROJECT_NAME}_test PROPERTIES TIMEOUT 60)
target_link_libraries(${PROJECT_NAME}_tests
    csapex_util
    gtest gtest_main)
