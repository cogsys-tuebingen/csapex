cmake_minimum_required(VERSION 2.8.3)
project(utils_vision)

set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS cv_bridge utils utils_plugin utils_param)

find_package(OpenCV REQUIRED)

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

find_package(Boost REQUIRED)
find_package(Boost COMPONENTS program_options serialization REQUIRED)

catkin_package(
    INCLUDE_DIRS src
    LIBRARIES utils_vision logger
    CATKIN_DEPENDS utils_plugin utils_param
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    src
)

##### COMMON
set(GLOBALS
    src/common/global.hpp)

##### LOGGER
add_library(logger
    src/common/global.hpp
    src/common/logger.cpp)

##### CORE
add_library(utils_vision
    src/utils/extractors_default.cpp

    src/config/config.cpp
    src/config/configurated_tools.cpp
    src/config/reconfigurable.cpp
    src/config/types.cpp
    src/data/angle.cpp
    src/data/directory_io.cpp
    src/data/frame.cpp
    src/data/frame_buffer.cpp
    src/data/frame_io.cpp
    src/data/matchable.cpp
    src/data/matchable_pose.cpp
    src/data/painter.cpp
    src/data/pose.cpp
    src/utils/extractor.cpp
    src/utils/extractor_factory.cpp
    src/utils/extractor_manager.cpp
    src/utils/hough_peak.cpp
    src/utils/matcher.cpp
    src/utils/match_scorer.cpp
    src/utils/match_scorer_homography.cpp
    src/utils/match_scorer_clustering.cpp
    src/utils/match_scorer_reprojection.cpp
    src/utils/match_scorer_factory.cpp
    src/utils/opencv_utils.hpp
    src/utils/grusig_descriptor.cpp
    src/utils/rectangle_cluster.cpp
    ${QT_UTIL}
    ${GLOBALS})
target_link_libraries(utils_vision yaml-cpp ${Boost_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES})
target_link_libraries(utils_vision logger)
