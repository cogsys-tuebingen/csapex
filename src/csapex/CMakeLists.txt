cmake_minimum_required(VERSION 3.8.2)

project(csapex)

## Find catkin macros and libraries
find_package(catkin QUIET)
find_package(Threads)

find_package(Qt5 5.0 REQUIRED COMPONENTS Core)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

qt5_add_resources(QT_RESOURCES res/csapex_resources.qrc)

## CATKIN
if(${catkin_FOUND})
    find_package(catkin REQUIRED COMPONENTS csapex_core csapex_remote csapex_qt)

    catkin_package(
       CATKIN_DEPENDS
          csapex_core
          csapex_remote
          csapex_qt
    )
else()
    message("building without catkin")

    set(Poco_DIR cmake)
    find_package(Poco REQUIRED COMPONENTS Foundation)
    find_package(console_bridge REQUIRED)
    set(INSTALL_DIR ${CMAKE_INSTALL_PREFIX})
    set(CATKIN_PACKAGE_INCLUDE_DESTINATION ${INSTALL_DIR}/include)
    set(CATKIN_PACKAGE_LIB_DESTINATION ${INSTALL_DIR}/lib)
    set(CATKIN_GLOBAL_BIN_DESTINATION ${INSTALL_DIR}/bin)
    set(CATKIN_PACKAGE_SHARE_DESTINATION ${INSTALL_DIR}/share)
    set(CSAPEX_MODULE_DESTINATION ${INSTALL_DIR}/CMake)

    set(CSAPEX_BOOT_PLUGIN_DIR ${CATKIN_GLOBAL_BIN_DESTINATION}/csapex_plugins)


    FILE(READ ${CMAKE_CURRENT_LIST_DIR}/package.xml package)

    string(REGEX MATCH "^.*<version>([^<]*)</version>.*$" match ${package})
    if(match)
        set(csapex_VERSION ${CMAKE_MATCH_1})
    endif()

    string(REGEX MATCH "^.*<maintainer.*email=\"[^\"]*\">([^<]*)</maintainer>.*$" match ${package})
    if(match)
        set(csapex_MAINTAINER ${CMAKE_MATCH_1})
    endif()

    if(WIN32)
        # disable min and max macros
        add_definitions(/D NOMINMAX)
    endif()
endif()


###########
## Build ##
###########

include_directories(
    ${catkin_INCLUDE_DIRS}
)


set(CMAKE_POSITION_INDEPENDENT_CODE ON)

#
# BUILD THE APPLICATION
#

if(WIN32)
        add_executable(csapex_main
                src/csapex.cpp
                res/csapex.rc
                ${QT_RESOURCES}
        )
else()
        add_executable(csapex_main
                src/csapex.cpp)
endif()

set_target_properties(csapex_main PROPERTIES
    AUTOMOC TRUE
    COMPILE_FLAGS "-fPIC")
target_link_libraries(csapex_main
    ${catkin_LIBRARIES}
    Threads::Threads
)



add_executable(csapex_server
        src/csapex_server.cpp)

target_link_libraries(csapex_server
    ${catkin_LIBRARIES}
    Threads::Threads
)



add_executable(csapex_client
        src/csapex_client.cpp

        ${QT_RESOURCES}
)

set_target_properties(csapex_client PROPERTIES
    AUTOMOC TRUE
    COMPILE_FLAGS "-fPIC")
target_link_libraries(csapex_client
    ${catkin_LIBRARIES}
    Threads::Threads
)


#
# CFG
#
add_custom_target(${PROJECT_NAME}_cfg ALL DEPENDS res/csapex.desktop)
add_custom_command(TARGET csapex_main POST_BUILD
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/setup.sh
        ${CATKIN_DEVEL_PREFIX}
        ${PROJECT_NAME}
        csapex_main
        csapex_server
        csapex_client
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
)


#
# INSTALL
#
install(TARGETS csapex_main
        DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(TARGETS csapex_client
        DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(TARGETS csapex_server
        DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
