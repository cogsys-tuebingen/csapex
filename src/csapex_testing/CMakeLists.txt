cmake_minimum_required(VERSION 3.8.2)

project(csapex_testing CXX)

enable_testing()

## Enforce that we use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS csapex)

catkin_package(
   INCLUDE_DIRS
      include
   LIBRARIES
      ${PROJECT_NAME}
   CATKIN_DEPENDS
      csapex
)

include_directories(${catkin_INCLUDE_DIRS})

#find_package(GTest REQUIRED)
include( CTest )

# framework files
file(GLOB framework_SRC
    "src/*.cpp"
)

# test framework
add_library(${PROJECT_NAME}
    ${framework_SRC}
)
target_include_directories(${PROJECT_NAME}
    PUBLIC
        include
        ${GTEST_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME}
    ${csapex_LIBRARIES}
    pthread
)

SET(ENABLE_COVERAGE "false" CACHE BOOL "Enable Coverage")

if(NOT ${ENABLE_COVERAGE})
    # regression tester
    add_executable(csapex_regression_tester
        src/regression_tester.cpp
    )
    add_test(NAME csapex_regression_test COMMAND csapex_regression_tester)
    set_tests_properties(csapex_regression_test PROPERTIES TIMEOUT 60)
    target_link_libraries(csapex_regression_tester
        ${catkin_LIBRARIES}
        pthread
    )
endif()

#
# INSTALL
#
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

