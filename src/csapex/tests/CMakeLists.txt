cmake_minimum_required(VERSION 2.8.11)

project(csapex_tests)

enable_testing()

SET(CMAKE_CXX_FLAGS "-g -O0 -fprofile-arcs -ftest-coverage")
SET(CMAKE_C_FLAGS "-g -O0 -fprofile-arcs -ftest-coverage")

## Enforce that we use C++11
if (CMAKE_VERSION VERSION_LESS "3.1")
  include(CheckCXXCompilerFlag)
  CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
  CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
  CHECK_CXX_COMPILER_FLAG("-std=gnu++11" COMPILER_SUPPORTS_GNU)
  if(COMPILER_SUPPORTS_CXX11)
     set (CMAKE_CXX_FLAGS "--std=c++11 ${CMAKE_CXX_FLAGS}")
  elseif(COMPILER_SUPPORTS_CXX0X)
     set (CMAKE_CXX_FLAGS "--std=c++0x ${CMAKE_CXX_FLAGS}")
  elseif(COMPILER_SUPPORTS_GNU)
     set (CMAKE_CXX_FLAGS "--std=gnu++11 ${CMAKE_CXX_FLAGS}")
  else()
     message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
  endif()
else ()
  set (CMAKE_CXX_STANDARD 11)
endif ()


#find_package(GTest REQUIRED)
include( CTest )

include_directories(${GTEST_INCLUDE_DIR} include)


# tests that are not yet distributed across the project
add_executable(${PROJECT_NAME}
    src/code_api_test.cpp
    src/auto_generate_test.cpp
    src/processing_test.cpp
    src/message_template_test.cpp
    src/graph_test.cpp
    src/polymorphic_msgs_test.cpp
    src/polymorphic_value_test.cpp
    src/node_creation_test.cpp
    src/connection_test.cpp
    src/signal_test.cpp
    src/transition_test.cpp
    src/yaml_serialization_test.cpp
    src/binary_serialization_test.cpp
    src/scheduling_test.cpp
    src/nesting_test.cpp
    src/parameter_test.cpp
    src/activity_test.cpp
    src/container_iteration_test.cpp
)

add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}
    csapex csapex_param csapex_util
    csapex_serialization
    gtest gtest_main)
