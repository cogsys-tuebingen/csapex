cmake_minimum_required(VERSION 2.8.3)
project(voxel_grid)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED rosconsole)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES voxel_grid
#   CATKIN_DEPENDS
#  DEPENDS system_lib
)

include_directories(include)

add_library(voxel_grid src/voxel_grid.cpp)
link_libraries(voxel_grid ${catkin_LIBRARIES})


add_executable(voxel_grid.exe src/voxel_grid_main.cpp)
link_libraries(voxel_grid.exe voxel_grid ${catkin_LIBRARIES})
