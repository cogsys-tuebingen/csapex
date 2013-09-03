cmake_minimum_required(VERSION 2.8.3)
project(csapex_vision_features)

add_definitions(-W -Wall -Wno-unused-parameter -fno-strict-aliasing -Wno-unused-function)

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
find_package(catkin REQUIRED COMPONENTS csapex csapex_vision utils_vision)

find_package(OpenCV REQUIRED)

find_package(Boost REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)

find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})


file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.h include/csapex_vision_features/*.h)

QT4_WRAP_CPP(QT_MOC ${QT_MOC})

set(QT ${QT_SOURCES} ${QT_MOC})


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES csapex_vision_features
#  CATKIN_DEPENDS csapex csapex_vision utils_vision
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(csapex_vision_features
    src/keypoint_message.cpp
    src/descriptor_message.cpp
    ${QT}
)
target_link_libraries(csapex_vision_features
    yaml-cpp ${QT_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_library(csapex_vision_features_plugin
    src/register_plugin.cpp
    src/extract_keypoints.cpp
    src/extract_descriptors.cpp
    src/display_keypoints.cpp
    ${QT}
)
target_link_libraries(csapex_vision_features_plugin csapex_vision_features
    yaml-cpp ${QT_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

