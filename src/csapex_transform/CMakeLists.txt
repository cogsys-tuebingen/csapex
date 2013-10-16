cmake_minimum_required(VERSION 2.8.3)
project(csapex_transform)


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
find_package(catkin REQUIRED COMPONENTS csapex csapex_core_plugins tf)

find_package(Boost REQUIRED)

find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})


file(GLOB_RECURSE QT_MOC_CORE RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/csapex_transform/*.h)
file(GLOB_RECURSE QT_MOC_PLUG RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.h)

QT4_WRAP_CPP(QT_MOC_CORE ${QT_MOC_CORE})
QT4_WRAP_CPP(QT_MOC_PLUG ${QT_MOC_PLUG})

set(QT_CORE ${QT_SOURCES}  ${QT_MOC_CORE})
set(QT_PLUG ${QT_SOURCES} ${QT_MOC_PLUG})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES csapex_transform
#  CATKIN_DEPENDS csapex csapex_core_plugins
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(csapex_transform
    src/transform_message.cpp
    src/time_stamp_message.cpp
    ${QT_CORE}
)
target_link_libraries(csapex_transform
    yaml-cpp ${QT_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_library(csapex_transform_plugin
    src/register_plugin.cpp
    src/clock.cpp
    src/time_offset.cpp
    src/static_transform.cpp
    src/dynamic_transform.cpp
    src/transform_combiner.cpp
    src/transform_inverter.cpp
    src/listener.cpp
    ${QT_PLUG}
)
target_link_libraries(csapex_transform_plugin csapex_transform
    yaml-cpp ${QT_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

