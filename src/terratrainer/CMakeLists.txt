cmake_minimum_required(VERSION 2.8.3)
project(terratrainer)

set (CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(catkin REQUIRED COMPONENTS utils)
## System dependencies are found with CMake's conventions
find_package(QT4    COMPONENTS QtCore QtGui REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost  COMPONENTS regex REQUIRED)

set (CMAKE_BUILD_TYPE Debug)
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
#  INCLUDE_DIRS include
#  LIBRARIES terratrainer
#  CATKIN_DEPENDS csapex csvision
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


### QT
include(${QT_USE_FILE})
file(GLOB         QT_FORMS   RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}                 ui/*.ui)
file(GLOB_RECURSE QT_MOC_GUI RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/gui/*.h)
file(GLOB_RECURSE QT_MOC_CTR RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/controllers/*.h)

QT4_WRAP_CPP(QT_MOC_GUI  ${QT_MOC_GUI})
QT4_WRAP_CPP(QT_MOC_CTR  ${QT_MOC_CTR})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_ADD_RESOURCES(QT_RESOURCES res/resources.qrc)

set(QT_TERRA ${QT_SOURCES} ${QT_FORMS_HPP} ${QT_MOC_GUI} ${QT_MOC_CTR} ${QT_RESOURCES})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_BINARY_DIR})



## Declare a cpp executable
add_executable(TerraTrainer
                src/computation/params.hpp
                src/computation/extractors.hpp
                src/computation/yaml.hpp
                src/computation/cmp_core.cpp
                src/computation/cmp_extractor.cpp
                src/computation/cmp_extractor_extended.cpp
                src/computation/cmp_randomforest.cpp
                src/computation/cmp_randomforest_extended.cpp
                src/controllers/ctrl_factory.cpp
                src/controllers/ctrl_main_menu.cpp
                src/controllers/ctrl_map_view.cpp
                src/controllers/ctrl_toolpanel.cpp
                src/controllers/ctrl_cmpcore_bridge.cpp
                src/controllers/ctrl_class_edit.cpp
                src/controllers/ctrl_preferences.cpp
                src/gui/main.cpp
                src/gui/terra_trainer_window.cpp
                src/gui/terra_sub_window.cpp
                src/graphics/qinteractive_rect.cpp
                src/graphics/qinteractive_scene.cpp
# ---------------------------
                src/common/QtCvImageConverter.cpp
# ---------------------------
                ${QT_TERRA})

target_link_libraries(TerraTrainer
                        yaml-cpp
                        ${QT_LIBRARIES}
                        ${OpenCV_LIBS}
                        ${Boost_LIBRARIES}
                        ${catkin_LIBRARIES})
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(terratrainer_node terratrainer_generate_messages_cpp)



#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS terratrainer terratrainer_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_terratrainer.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
