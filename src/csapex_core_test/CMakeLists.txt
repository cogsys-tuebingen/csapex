cmake_minimum_required(VERSION 3.8.2)

project(csapex_core_test CXX)

enable_testing()

## Enforce that we use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS csapex_testing)

csapex_package()

include_directories(${catkin_INCLUDE_DIRS})

include( CTest )

# tests that are not yet distributed across the project
file(GLOB_RECURSE tests_SRC
    "src/*.cpp"
)

catkin_add_gtest(${PROJECT_NAME}
   ${tests_SRC}
)
# set_tests_properties(${PROJECT_NAME} PROPERTIES TIMEOUT 60)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    gtest gtest_main)
