cmake_minimum_required(VERSION 2.8.3)
project(csapex_bar_code)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(ZBar)

if(${ZBAR_FOUND})

    ## Find catkin macros and libraries
    ## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
    ## is used, also find other catkin packages
    find_package(catkin REQUIRED COMPONENTS csapex csapex_vision)

    find_package(OpenCV REQUIRED)


    csapex_moc_all(QT_MOC src/*.h)

    set(QT ${QT_SOURCES} ${QT_MOC})


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
      LIBRARIES csapex_vision_features
    #  CATKIN_DEPENDS csapex csapex_vision utils_vision
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
      ${ZBAR_INCLUDE_DIR}
    )

    add_library(csapex_bar_code_plugin
        src/bar_code_reader.cpp
        ${QT}
    )
    target_link_libraries(csapex_bar_code_plugin
        yaml-cpp ${QT_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES} ${ZBAR_LIBRARIES})

endif(${ZBAR_FOUND})
