cmake_minimum_required(VERSION 3.0.2)

project(csapex_testing)

enable_testing()

SET(CMAKE_CXX_FLAGS "-g -O0 -fprofile-arcs -ftest-coverage")
SET(CMAKE_C_FLAGS "-g -O0 -fprofile-arcs -ftest-coverage")

## Enforce that we use C++11
if (CMAKE_VERSION VERSION_LESS "3.1")
  include(CheckCXXCompilerFlag)
  CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
  CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
  CHECK_CXX_COMPILER_FLAG("-std=gnu++11" COMPILER_SUPPORTS_GNU)
  if(COMPILER_SUPPORTS_CXX11)
     set (CMAKE_CXX_FLAGS "--std=c++11 ${CMAKE_CXX_FLAGS}")
  elseif(COMPILER_SUPPORTS_CXX0X)
     set (CMAKE_CXX_FLAGS "--std=c++0x ${CMAKE_CXX_FLAGS}")
  elseif(COMPILER_SUPPORTS_GNU)
     set (CMAKE_CXX_FLAGS "--std=gnu++11 ${CMAKE_CXX_FLAGS}")
  else()
     message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
  endif()
else ()
  set (CMAKE_CXX_STANDARD 11)
endif ()

find_package(catkin REQUIRED COMPONENTS csapex)

catkin_package(
   INCLUDE_DIRS
      include
   LIBRARIES
      ${PROJECT_NAME}
   CATKIN_DEPENDS
      csapex
)

include_directories(${catkin_INCLUDE_DIRS})

#find_package(GTest REQUIRED)
include( CTest )

# framework files
file(GLOB framework_SRC
    "src/*.cpp"
)

# test framework
add_library(${PROJECT_NAME}
    ${framework_SRC}
)
target_include_directories(${PROJECT_NAME}
    PUBLIC
        include
        ${GTEST_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME}
    ${csapex_LIBRARIES}
)

#
# INSTALL
#
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
