cmake_minimum_required(VERSION 2.8.3)
project(utils_param)

set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED)

find_package(Boost REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES utils_param
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)


##### CORE
add_library(utils_param
    src/io.cpp
    src/parameter.cpp
    src/range_parameter.cpp
    src/set_parameter.cpp
    src/value_parameter.cpp
    src/path_parameter.cpp
    src/parameter_factory.cpp
    src/parameter_map.cpp
    src/parameter_provider.cpp)
target_link_libraries(utils_param yaml-cpp ${Boost_LIBRARIES} )
