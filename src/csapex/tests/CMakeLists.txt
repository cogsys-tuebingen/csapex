cmake_minimum_required(VERSION 2.8.11)

set(PROJECT_TEST_NAME csapex_tests)

enable_testing()

SET(CMAKE_CXX_FLAGS "-g -O0 -fprofile-arcs -ftest-coverage")
SET(CMAKE_C_FLAGS "-g -O0 -fprofile-arcs -ftest-coverage")

#find_package(GTest REQUIRED)
include( CTest )

include_directories(${GTEST_INCLUDE_DIR} include)

add_executable(csapex_test
    src/tests.cpp

    src/code_api_test.cpp
    src/auto_generate_test.cpp

    src/graph_test.cpp
    src/node_creation_test.cpp
    src/connection_test.cpp
    src/signal_test.cpp
    src/transition_test.cpp
    src/uuid_test.cpp
    src/yaml_serialization_test.cpp
    src/binary_serialization_test.cpp
    src/slim_signals_test.cpp
    src/nesting_test.cpp
    src/parameter_test.cpp
    src/activity_test.cpp
    src/container_iteration_test.cpp
)

add_test(NAME ${PROJECT_TEST_NAME} COMMAND csapex_test)

target_link_libraries(csapex_test
    csapex csapex_param csapex_util
    csapex_serialization
    gtest gtest_main)
