cmake_minimum_required(VERSION 2.8.3)
project(laser_geometry)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread)

find_package(Eigen)
include_directories(${Eigen_INCLUDE_DIRS})

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES laser_geometry
#   CATKIN_DEPENDS
#  DEPENDS system_lib
)

include_directories(include)

add_library(laser_geometry src/laser_geometry.cpp )
target_link_libraries(laser_geometry ${Boost_LIBRARIES})

