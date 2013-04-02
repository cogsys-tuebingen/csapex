cmake_minimum_required(VERSION 2.8.3)
project(rabot_utils)

find_package(catkin REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIR})

find_package(Utils REQUIRED)
include_directories(${UTILS_INCLUDE_DIRS})

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#   INCLUDE_DIRS ${UTILS_INCLUDE_DIRS}
#   LIBRARIES ${UTILS_LIBRARIES_VERBOSE}
#  CATKIN_DEPENDS
#  DEPENDS system_lib
)

#foreach(lib ${UTILS_LIBRARY_PATHS})
#    message(ERROR ${lib})
#    execute_process(COMMAND ln -s ${UTILS_PATH}/src/${lib} ${CMAKE_CURRENT_SOURCE_DIR}/src
#                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
#endforeach()

#execute_process(COMMAND ln -s ${UTILS_PATH}/src/CMakeLists.txt ${CMAKE_CURRENT_SOURCE_DIR}/src
#                WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
execute_process(COMMAND ln -s ${UTILS_PATH}/src ${CMAKE_CURRENT_SOURCE_DIR}
                WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(src)
