cmake_minimum_required(VERSION 2.8.3)
project(csapex_vision)

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
find_package(catkin REQUIRED COMPONENTS pluginlib roscpp rospy std_msgs rosbag cv_bridge  utils csapex csapex_core_plugins)

find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)

find_package(Boost REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)

find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})


file(GLOB         QT_FORMS RELATIVE   ${CMAKE_CURRENT_SOURCE_DIR}                 ui/*.ui)
file(GLOB_RECURSE QT_MOC_EVA RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/evaluator/*.h include/csapex_vision/*.h)
file(GLOB_RECURSE QT_MOC_UTI RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/utils/*.h)

QT4_WRAP_CPP(QT_MOC_EVA ${QT_MOC_EVA})
QT4_WRAP_CPP(QT_MOC_DES ${QT_MOC_DES})
QT4_WRAP_CPP(QT_MOC_UTI ${QT_MOC_UTI})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_ADD_RESOURCES(QT_RESOURCES res/csapex_vision_resources.qrc)

set(QT_EVALUATOR ${QT_SOURCES} ${QT_FORMS_HPP} ${QT_MOC_EVA})
set(QT_UTIL ${QT_SOURCES} ${QT_FORMS_HPP} ${QT_MOC_UTI})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include src
   LIBRARIES logger csapex_vision
   CATKIN_DEPENDS pluginlib roscpp std_msgs csapex utils_plugin
   DEPENDS Qt4 OpenCV
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
)


add_library(csapex_vision
    src/evaluator/filter.cpp
    src/evaluator/image_combiner.cpp
    src/evaluator/image_provider.cpp
    src/evaluator/image_provider_bag.cpp
    src/evaluator/image_provider_dir.cpp
    src/evaluator/image_provider_img.cpp
    src/evaluator/image_provider_mov.cpp
    src/evaluator/image_provider_set.cpp
    src/evaluator/register_plugin.cpp
    src/evaluator/filter_blur.cpp
    src/evaluator/filter_apply_mask.cpp
    src/evaluator/filter_static_mask.cpp
    src/evaluator/image_combiner_robust_match.cpp
    src/evaluator/output_display.cpp
    src/evaluator/cv_mat_message.cpp

    ${QT_EVALUATOR}
    ${QT_RESOURCES}
)
target_link_libraries(csapex_vision
    yaml-cpp ${QT_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${OPENGL_LIBRARIES} ${catkin_LIBRARIES})


add_executable(vision_evaluator src/deprecated.cpp)

