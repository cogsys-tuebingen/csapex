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
find_package(catkin REQUIRED COMPONENTS pluginlib roscpp rospy std_msgs rosbag cv_bridge)

find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIRS})

find_package(Utils REQUIRED)
include_directories(${UTILS_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)

find_package(Boost REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)

find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})


file(GLOB         QT_FORMS RELATIVE         ${CMAKE_CURRENT_SOURCE_DIR}                 ui/*.ui)
file(GLOB_RECURSE QT_MOC_EVA RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/evaluator/*.h)
QT4_WRAP_CPP(QT_MOC_EVA ${QT_MOC_EVA})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
set(QT_EVALUATOR ${QT_SOURCES} ${QT_FORMS_HPP} ${QT_MOC_EVA})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES vision_evaluator
  CATKIN_DEPENDS pluginlib roscpp std_msgs rabot_utils
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

##### EVALUATION
add_executable(evaluator
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
target_link_libraries(evaluator ${QT_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${OPENGL_LIBRARIES} ${catkin_LIBRARIES})

# INSTALL INCLUDE HEADERS
string(REGEX REPLACE ".*/src/([^/]+)" "\\1" lib ${CMAKE_CURRENT_SOURCE_DIR})
file(GLOB_RECURSE headers RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.h *.hpp)
foreach(h ${headers})
    get_filename_component(dir ${h} PATH)
    file(COPY ${h} DESTINATION ${PROJECT_SOURCE_DIR}/include/${lib}/)
    #message("file(COPY ${h} DESTINATION ${PROJECT_SOURCE_DIR}/include/${lib}/)")
endforeach()

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS vision_evaluator vision_evaluator_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_vision_evaluator.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
