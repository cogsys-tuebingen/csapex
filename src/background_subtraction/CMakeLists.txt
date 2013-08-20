cmake_minimum_required(VERSION 2.8.3)
project(background_subtraction)

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
find_package(catkin REQUIRED COMPONENTS  genmsg roscpp sensor_msgs cv_bridge dynamic_reconfigure utils csapex csapex_vision)

find_package(OpenCV REQUIRED)

find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
set(QT_MOC_BG_REMOVE src/background_remover_window.h src/background_remover_node.h)
QT4_WRAP_CPP(QT_MOC_HPP_BG_REMOVE ${QT_MOC_BG_REMOVE})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
set(QT ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP}   ${QT_MOC_HPP_BG_REMOVE} )


set(QT_MOC_PLUGIN src/background_subtraction.h)
QT4_WRAP_CPP(QT_PLUGIN ${QT_MOC_PLUGIN})

file(GLOB INCLUDES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} include/background_subtraction/*.h)
set(headers ${INCLUDES})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(cfg/Global.cfg)


# for generated config
include_directories(cfg/cpp)

# for generated qt stuff
include_directories(${CMAKE_BINARY_DIR}/${PROJECT_NAME})


## Find message files
file(GLOB messages messages RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/msg/"
    "${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg")

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  ${messages}
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES bg_sub
   CATKIN_DEPENDS message_runtime sensor_msgs
#  DEPENDS system_lib
)



file(COPY src/masked_image_handler.h DESTINATION ${PROJECT_SOURCE_DIR}/include/background_subtraction/)
include_directories(include
  ${catkin_INCLUDE_DIRS}
)



add_library(bg_sub
    src/masked_image_handler.cpp
    ${headers}
)
target_link_libraries(bg_sub
    ${QT_LIBRARIES}
    ${catkin_LIBRARIES}
)

set(bg_remover_src
    src/node.cpp
    src/adaptive_background_remover.cpp
    src/background_remover.cpp
    src/background_remover_node.cpp
    src/background_remover_window.cpp
    src/simple_background_remover.cpp
    src/opencv_background_remover.cpp
    ${QT}
)


set(bitness 32)
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(bitness 64)
endif()

if(bitness EQUAL 64)
    set(bg_remover_src
        ${bg_remover_src}
        src/vibe_background_remover.cpp
        src/vibe-background.o
    )
endif()

add_executable(bg_remover ${bg_remover_src})

target_link_libraries(bg_remover
    bg_sub
    ${QT_LIBRARIES}
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
)

add_executable(masked2img
    src/masked_2_img_node.cpp
)
target_link_libraries(masked2img
    bg_sub
    ${catkin_LIBRARIES}
)

add_library(bgsubplugin
    src/background_subtraction.cpp
    src/csapex_plugin.cpp
    src/background_remover.cpp
    src/adaptive_background_remover.cpp
    src/simple_background_remover.cpp
    src/opencv_background_remover.cpp
    ${QT_PLUGIN}
)
target_link_libraries(bgsubplugin
    bg_sub
    ${catkin_LIBRARIES}
)
