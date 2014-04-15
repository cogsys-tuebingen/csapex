cmake_minimum_required(VERSION 2.8.3)
project(csapex_boolean)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
find_package(catkin REQUIRED COMPONENTS csapex csapex_core_plugins)

csapex_moc_all(QT_MOC src/*.h)

QT4_ADD_RESOURCES(QT_RESOURCES res/csapex_boolean_resources.qrc)

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
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(csapex_boolean_plugin
    src/register_plugin.cpp
    src/nand.cpp
    src/toggle.cpp
    src/indicator.cpp

    ${QT_MOC}
    ${QT_RESOURCES}
)
target_link_libraries(csapex_boolean_plugin
    yaml-cpp ${QT_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES})
