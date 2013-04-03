cmake_minimum_required(VERSION 2.8.3)
project(robot_detection)

add_definitions(-W -Wall -Wno-unused-parameter -fno-strict-aliasing -Wno-unused-function)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(CMAKE_BUILD_TYPE Debug)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs cv_bridge dynamic_reconfigure tf vision_evaluator background_subtraction tf_conversions)

find_package(Utils REQUIRED)
include_directories(${UTILS_INCLUDE_DIRS})

find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui QtOpenGL)
include(${QT_USE_FILE})

find_package(Boost COMPONENTS system filesystem serialization iostreams signals program_options thread REQUIRED)

find_package(ViolaJones REQUIRED)
include_directories(${VIOLA_JONES_INCLUDE_DIRS})

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(DBOW2 REQUIRED)
include_directories(${DBOW2_INCLUDE_DIRS})

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)


generate_dynamic_reconfigure_options(cfg/Global.cfg cfg/TfCreator.cfg)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/cfg/cpp/)
include_directories(${PROJECT_SOURCE_DIR}/src)

# for generated qt stuff
include_directories(${CMAKE_BINARY_DIR}/${PROJECT_NAME})

ADD_DEFINITIONS(-DQT_NO_KEYWORDS)

file(GLOB         QT_FORMS RELATIVE         ${CMAKE_CURRENT_SOURCE_DIR}                 ui/*.ui)
file(GLOB         QT_RESOURCES RELATIVE     ${CMAKE_CURRENT_SOURCE_DIR}                 resources/*.qrc)
file(GLOB_RECURSE QT_MOC_ROB RELATIVE       ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/viz/*.h)
file(GLOB_RECURSE QT_MOC_EVA RELATIVE       ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/evaluator/*.h)
QT4_ADD_RESOURCES( QT_RESOURCES_CPP     ${QT_RESOURCES})
QT4_WRAP_UI(       QT_FORMS_HPP         ${QT_FORMS})
QT4_WRAP_CPP(      QT_MOC_EVA           ${QT_MOC_EVA})
QT4_WRAP_CPP(      QT_MOC_ROB           ${QT_MOC_ROB})

set(QT_EVALUATOR ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_EVA})
set(QT_ROBOTFINDER ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_ROB})


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_definitions(-ggdb)

########################
########################
##### LIBRARIES


##### COMMON
set(GLOBALS                         src/common/global.hpp)

##### LOGGER
add_library(logger                  src/utils/logger.cpp)

##### CONFIG
add_library(cfg                     src/config/config.cpp
                                    src/config/configurated_tools.cpp
                                    src/config/reconfigurable.cpp
                                    src/config/types.cpp
                                    ${GLOBALS})
target_link_libraries(cfg           logger)

##### DATA
add_library(data                    src/data/angle.cpp
                                    src/data/directory_io.cpp
                                    src/data/frame.cpp
                                    src/data/frame_buffer.cpp
                                    src/data/frame_io.cpp
                                    src/data/matchable.cpp
                                    src/data/matchable_pose.cpp
                                    src/data/painter.cpp
                                    src/data/pose.cpp
                                    ${GLOBALS})
target_link_libraries(data          yaml-cpp ${Boost_LIBRARIES} ${UTILS_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(data          logger)

##### ROC
add_library(roc                     src/roc/roc_creator.cpp
                                    ${GLOBALS})
target_link_libraries(roc           yaml-cpp ${Boost_LIBRARIES} ${OpenCV_LIBS})

##### UTILS
add_library(utils                   src/utils/extractor.cpp
                                    src/utils/extractor_factory.cpp
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
target_link_libraries(utils         data)
target_link_libraries(utils         yaml-cpp ${Boost_LIBRARIES} ${OpenCV_LIBS})

##### DATABASE
add_library(database                src/db/bag.cpp
                                    src/db/bag_database.cpp
                                    src/db/bin.cpp
                                    src/db/bin_database.cpp
                                    src/db/bow_database.cpp
                                    src/db/database.cpp
                                    src/db/database_io.cpp
                                    src/db/naive_database.cpp
                                    src/db/serializer.h
                                    ${GLOBALS})
target_link_libraries(database      data)
target_link_libraries(database      ${DBOW2_LIBRARIES} ${Boost_LIBRARIES})

##### DATABASE STRATEGIES
add_library(db_strategy             src/db_strategy/bin_db_strategy.cpp
                                    src/db_strategy/bag_db_strategy.cpp
                                    src/db_strategy/bow_db_strategy.cpp
                                    src/db_strategy/db_strategy.cpp
                                    src/db_strategy/db_strategy_interface.cpp
                                    src/db_strategy/db_strategy_decorator.cpp
                                    src/db_strategy/evaluation_strategy_decorator.cpp
                                    src/db_strategy/factory.cpp
                                    src/db_strategy/naive_db_strategy.cpp
                                    ${GLOBALS})
target_link_libraries(db_strategy   database roc)

##### ROI
add_library(roi                     src/roi/roi.cpp
                                    src/roi/roi_manager.cpp
                                    src/roi/roi_provider.cpp
                                    src/roi/nth_full_frame_roi.cpp
                                    src/roi/screen_space_roi_tracker.cpp
                                    src/roi/viola_jones_roi.cpp
                                    ${GLOBALS})
target_link_libraries(roi           utils)


##### ANALYZER
add_library(analyzer                src/analyzer/analyzer.cpp
                                    src/analyzer/detector.cpp
                                    src/analyzer/dumper.cpp
                                    src/analyzer/trainer.cpp
                                    src/analyzer/trainer_online.cpp
                                    ${GLOBALS})
target_link_libraries(analyzer      data roi db_strategy)
target_link_libraries(analyzer      ${VIOLA_JONES_LIBRARIES} ${OpenCV_LIBS})

##### TRACKER
add_library(tracker                 src/tracker/tracker.cpp
                                    src/tracker/kalman_filter.cpp
                                    src/tracker/hypothesis.cpp
                                    ${GLOBALS})
target_link_libraries(tracker       data)

##### EVALUATION
add_library(evaluation_plugin       src/evaluator/filter_display_features.cpp
                                    src/evaluator/histogram_viewer_widget.cpp
                                    src/evaluator/image_combiner_cluster_match.cpp
                                    src/evaluator/image_combiner_cluster_match_em.cpp
                                    src/evaluator/image_combiner_peak_match.cpp
                                    src/evaluator/image_combiner_simple_match.cpp
                                    src/evaluator/option_clustering.cpp
                                    src/evaluator/option_keypoint_extractor.cpp
                                    src/evaluator/option_keypoint_descriptor.cpp
                                    ${QT_EVALUATOR}
)

target_link_libraries(evaluation_plugin
                                    logger cfg utils data Random ${catkin_LIBRARIES} ${UTILS_LIBRARIES}
                                    ${QT_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${OPENGL_LIBRARIES}  ${GLUT_LIBRARY})

##### ADAPTER
add_library(adapter                 src/adapter/analyzer_adapter.cpp
                                    src/adapter/detector_adapter.cpp
                                    src/adapter/detector_adapter_static.cpp
                                    src/adapter/trainer_adapter.cpp
                                    src/adapter/trainer_adapter_static.cpp
                                    ${GLOBALS})
target_link_libraries(adapter       cfg analyzer logger)
target_link_libraries(adapter       ${Boost_LIBRARIES} ${QT_LIBRARIES})

##### VISUALIZATION
add_library(viz                     src/viz/analyzer_window.cpp
                                    src/viz/database_item_model_decorator.cpp
                                    src/viz/database_io_window.cpp
                                    src/viz/database_viewer.cpp
                                    ${QT_ROBOTFINDER}
                                    ${GLOBALS})
target_link_libraries(viz           db_strategy adapter ${QT_LIBRARIES} ${OpenCV_LIBS})


##### ROS DEPENDENT STUFF
add_library(ros                     src/ros/ros_adapter.cpp
                                    src/ros/ros_config.cpp
                                    src/ros/detector_adapter_ros.cpp
                                    src/ros/laser_background_subtractor.cpp
                                    src/ros/trainer_adapter_ros.cpp
                                    src/ros/tracker_adapter_ros.cpp
                                    ${GLOBALS})
target_link_libraries(ros           adapter cfg tracker ${catkin_LIBRARIES})


########################
########################
##### NODES AND PROGRAMS
add_executable(manual_tf_creator src/manual_tf_creator.cpp)
target_link_libraries(manual_tf_creator logger ${catkin_LIBRARIES})

#add_executable(antifilter_test src/anti_filter_test.cpp)
#target_link_libraries(antifilter_test   utils viz ${catkin_LIBRARIES})

#add_executable(test_suite      src/test_suite.cpp)
#target_link_libraries(test_suite        viz ${catkin_LIBRARIES})

add_executable(merge_test      src/merge_test.cpp)
target_link_libraries(merge_test        utils cfg ${catkin_LIBRARIES})

#add_executable(test_rectangle  src/test/test_rectangle.cpp)
#target_link_libraries(test_rectangle    utils ${catkin_LIBRARIES})

#add_executable(stats_feat_dist src/stats_feat_per_distance.cpp)
#target_link_libraries(stats_feat_dist   utils viz ${catkin_LIBRARIES})

add_executable(rerouter        src/rerouter.cpp)
target_link_libraries(rerouter          logger ${catkin_LIBRARIES})

add_executable(tf_importer     src/tf_importer.cpp)
target_link_libraries(tf_importer       logger ${catkin_LIBRARIES})

add_executable(pose2tf         src/pose_2_tf.cpp)
target_link_libraries(pose2tf           logger ${catkin_LIBRARIES})

add_executable(roc_viewer      src/roc_viewer.cpp)
target_link_libraries(roc_viewer        roc logger ${catkin_LIBRARIES})

add_executable(database_viewer src/database_viewer_node.cpp)
target_link_libraries(database_viewer   viz ${catkin_LIBRARIES})

add_executable(detector_online src/robot_detector_node.cpp)
target_link_libraries(detector_online   ros viz ${catkin_LIBRARIES})

add_executable(tracker_node    src/tracker_node.cpp)
target_link_libraries(tracker_node      ros viz ${catkin_LIBRARIES})

add_executable(detector_batch           src/robot_detector_batch.cpp)
target_link_libraries(detector_batch    viz)

#add_executable(thresh_norm     src/threshold_normalizer.cpp)
#target_link_libraries(thresh_norm       robot_finder ${catkin_LIBRARIES})

add_executable(trainer_online  src/robot_trainer_node.cpp)
target_link_libraries(trainer_online    ros viz)

add_executable(trainer_batch   src/robot_trainer_batch.cpp)
target_link_libraries(trainer_batch     viz)

add_executable(trainer_dump    src/robot_trainer_dump_node.cpp)
target_link_libraries(trainer_dump      ros viz ${catkin_LIBRARIES})




# analysis tools
get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)

set(directories "")
foreach(dir ${dirs})
  #message(STATUS "dir='${dir}'")
  set(directories "-I${dir} ${directories}")
endforeach()

add_custom_command(OUTPUT includes.txt
                   COMMAND eval "echo ${directories} >includes.txt")
add_custom_target(generate_includes ALL DEPENDS includes.txt)
