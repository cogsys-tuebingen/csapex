cmake_minimum_required(VERSION 2.8.3)
project(csapex_point_cloud)

add_definitions(-W -Wall -Wno-unused-parameter -fno-strict-aliasing -Wno-unused-function)
ADD_DEFINITIONS(-DQT_NO_KEYWORDS)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
set(CMAKE_BUILD_TYPE Debug)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS csapex csapex_core_plugins csapex_transform pcl_ros)

find_package(Boost REQUIRED)

find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})


file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.h include/csapex_point_cloud/*.h)

QT4_WRAP_CPP(QT_MOC ${QT_MOC})

set(QT ${QT_SOURCES} ${QT_MOC})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES csapex_point_cloud
#  CATKIN_DEPENDS csapex csapex_core_plugins
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(csapex_point_cloud
    src/point_cloud_message.cpp
    ${QT}
)
target_link_libraries(csapex_point_cloud
    yaml-cpp ${QT_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_library(csapex_point_cloud_plugin
    src/register_plugin.cpp
    src/point_count.cpp
    src/transform_cloud.cpp
    src/voxel_grid.cpp
    src/extract_timestamp.cpp
    src/set_timestamp.cpp
    ${QT}
)
target_link_libraries(csapex_point_cloud_plugin csapex_core_plugins
    yaml-cpp ${QT_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

