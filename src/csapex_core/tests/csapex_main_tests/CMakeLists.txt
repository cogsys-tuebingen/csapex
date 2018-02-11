cmake_minimum_required(VERSION 3.0.2)

project(csapex_main_tests)

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

find_package(catkin REQUIRED)

#find_package(GTest REQUIRED)
include( CTest )

# tests that are not yet distributed across the project
file(GLOB tests_SRC
    "src/*.cpp"
)

add_executable(${PROJECT_NAME}
    ${tests_SRC}
)
add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}
    csapex_test
    gtest gtest_main)
