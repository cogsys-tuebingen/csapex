cmake_minimum_required(VERSION 2.8.3)
project(utils)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED COMPONENTS genmsg roscpp sensor_msgs std_msgs costmap_2d_extended)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIR})

find_package(OpenCV REQUIRED)

file(GLOB messages messages RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/msg/"
    "${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg")
add_message_files(
  FILES
  ${messages}
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS
     include 
     include/utils/LibRandom
     include/utils/LibUtil/
     include/utils/LibBotHardware/
     include/utils/LibRobot/
     include/utils/LibPath/
     include/utils/LibLaserProcessing/
     include/utils/LibOdoCalib/
     include/utils/LibCvTools/
#     include/utils/LibLaserProcessing/segmentation
#     include/utils/LibLaserProcessing/leg_filter
   LIBRARIES
     Gnuplot
     LibUtil
     Random
     LibBotHardware 
     LibRobot
     LibMlsmap
     LibPath 
     LibGps 
     LibOdoCalib 
     LibLaserProcessing
     LibRosUtil
     LibCvTools
   CATKIN_DEPENDS genmsg roscpp sensor_msgs std_msgs costmap_2d_extended
   DEPENDS Eigen OpenCV
)

include_directories(include
	${catkin_INCLUDE_DIRS}
)

add_subdirectory(src)
