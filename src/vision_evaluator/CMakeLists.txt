cmake_minimum_required(VERSION 2.8.3)
project(vision_evaluator)

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
find_package(catkin REQUIRED COMPONENTS pluginlib roscpp rospy std_msgs rosbag cv_bridge  utils)

find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)

find_package(Boost REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)

find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})


file(GLOB         QT_FORMS RELATIVE   ${CMAKE_CURRENT_SOURCE_DIR}                 ui/*.ui)
file(GLOB_RECURSE QT_MOC_EVA RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/evaluator/*.h)
file(GLOB_RECURSE QT_MOC_DES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/designer/*.h)
QT4_WRAP_CPP(QT_MOC_EVA ${QT_MOC_EVA})
QT4_WRAP_CPP(QT_MOC_DES ${QT_MOC_DES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
set(QT_EVALUATOR ${QT_SOURCES} ${QT_FORMS_HPP} ${QT_MOC_EVA})
set(QT_DESIGNER ${QT_SOURCES} ${QT_FORMS_HPP} ${QT_MOC_DES})

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
#  LIBRARIES vision_evaluator
  CATKIN_DEPENDS pluginlib roscpp std_msgs utils
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
)


##### COMMON
set(GLOBALS
    src/common/global.hpp)

##### LOGGER
add_library(logger
    src/common/global.hpp
    src/common/logger.cpp)

##### CONFIG
add_library(cfg
    src/config/config.cpp
    src/config/configurated_tools.cpp
    src/config/reconfigurable.cpp
    src/config/types.cpp
    ${GLOBALS})
target_link_libraries(cfg
    logger)

##### DATA
add_library(data
    src/data/angle.cpp
    src/data/directory_io.cpp
    src/data/frame.cpp
    src/data/frame_buffer.cpp
    src/data/frame_io.cpp
    src/data/matchable.cpp
    src/data/matchable_pose.cpp
    src/data/painter.cpp
    src/data/pose.cpp
    ${GLOBALS})
target_link_libraries(data
    yaml-cpp ${Boost_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES})
target_link_libraries(data
    logger)

##### UTILS
add_library(utils
    src/utils/extractor.cpp
    src/utils/extractor_factory.cpp
    src/utils/extractor_manager.cpp
    src/utils/extractors_default.cpp
    src/utils/hough_peak.cpp
    src/utils/matcher.cpp
    src/utils/match_scorer.cpp
    src/utils/match_scorer_homography.cpp
    src/utils/match_scorer_clustering.cpp
    src/utils/match_scorer_reprojection.cpp
    src/utils/match_scorer_factory.cpp
    src/utils/opencv_utils.hpp
    src/utils/rectangle_cluster.cpp
    ${GLOBALS})
target_link_libraries(utils
    data)
target_link_libraries(utils
    yaml-cpp ${Boost_LIBRARIES} ${OpenCV_LIBS})

##### EVALUATION
add_library(designer
    src/designer/box.cpp
    src/designer/box_manager.cpp
    src/designer/connector.cpp
    src/designer/connector_in.cpp
    src/designer/connector_out.cpp
    src/designer/design_board.cpp
    src/designer/designer.cpp
    src/designer/overlay.cpp
    src/designer/selector_proxy.cpp
    ${QT_DESIGNER}
)

add_executable(vision_evaluator
    src/evaluator.cpp
    src/evaluator/evaluation_window.cpp
    src/evaluator/filter.cpp
    src/evaluator/filter_manager.cpp
    src/evaluator/filter_blur.cpp
    src/evaluator/filter_static_mask.cpp
    src/evaluator/generic_manager.hpp
    src/evaluator/image_combiner.cpp
    src/evaluator/image_combiner_manager.cpp
    src/evaluator/image_combiner_robust_match.cpp
    src/evaluator/image_combiner_panel.cpp
    src/evaluator/image_panel.cpp
    src/evaluator/image_provider.cpp
    src/evaluator/image_provider_bag.cpp
    src/evaluator/image_provider_dir.cpp
    src/evaluator/image_provider_img.cpp
    src/evaluator/image_provider_mov.cpp
    src/evaluator/image_provider_set.cpp
    src/evaluator/image_provider_ros_topic.cpp
    src/evaluator/global_option.cpp
    src/evaluator/global_option_manager.cpp
    src/evaluator/panel.cpp
    src/evaluator/viewer.cpp
    src/qt_helper.hpp
    ${QT_EVALUATOR}
)
target_link_libraries(vision_evaluator
    designer
    ${QT_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${OPENGL_LIBRARIES} ${catkin_LIBRARIES})

# INSTALL INCLUDE HEADERS
string(REGEX REPLACE ".*/src/([^/]+)" "\\1" lib ${CMAKE_CURRENT_SOURCE_DIR})
file(GLOB_RECURSE headers RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.h *.hpp)
foreach(h ${headers})
    get_filename_component(dir ${h} PATH)
    file(COPY ${h} DESTINATION ${PROJECT_SOURCE_DIR}/include/${lib}/)
    #message("file(COPY ${h} DESTINATION ${PROJECT_SOURCE_DIR}/include/${lib}/)")
endforeach()
