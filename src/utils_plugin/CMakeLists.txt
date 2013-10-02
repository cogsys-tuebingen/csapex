cmake_minimum_required(VERSION 2.8.3)
project(utils_plugin)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS pluginlib)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES utils_plugin
  CATKIN_DEPENDS pluginlib
#  DEPENDS system_lib
)

include_directories(include)

add_library(utils_plugin
    include/utils_plugin/singleton.hpp
    include/utils_plugin/plugin_manager.hpp
    include/utils_plugin/constructor.hpp
    src/plugin_loader.cpp
)
set_target_properties(utils_plugin PROPERTIES LINKER_LANGUAGE CXX)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
