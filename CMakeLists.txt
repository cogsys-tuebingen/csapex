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

set(YAML_LIBRARIES -L/usr/local/lib/ yaml-cpp)

##### CORE
add_library(utils_param
    src/io.cpp
    src/parameter.cpp
    src/parameter_description.cpp

    src/range_parameter.cpp
    src/interval_parameter.cpp
    src/set_parameter.cpp
    src/bitset_parameter.cpp
    src/value_parameter.cpp
    src/path_parameter.cpp
    src/trigger_parameter.cpp
    src/color_parameter.cpp
    src/null_parameter.cpp

    src/parameter_factory.cpp
    src/parameter_map.cpp
    src/parameter_provider.cpp)
target_link_libraries(utils_param ${YAML_LIBRARIES} ${Boost_LIBRARIES} )



install(TARGETS utils_param
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

