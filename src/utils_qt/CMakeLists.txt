cmake_minimum_required(VERSION 2.8.3)
project(utils_qt)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES utils_qt
  #CATKIN_DEPENDS pluginlib
#  DEPENDS system_lib
)

find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})

set(MOC_HDRS
    include/utils_qt/qxtspanslider.h
    include/utils_qt/qxtspanslider_p.h
)

qt4_wrap_cpp(MOC_SRCS ${MOC_HDRS})

include_directories(include)

add_library(utils_qt
    src/qxtspanslider.cpp
    src/QtCvImageConverter.cpp
    ${MOC_SRCS}
)

target_link_libraries(utils_qt
    ${catkin_LIBRARIES}
    ${QT_LIBRARIES})

