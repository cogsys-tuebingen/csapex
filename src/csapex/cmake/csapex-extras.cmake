add_definitions(-W -Wall -Wno-unused-parameter -fno-strict-aliasing -Wno-unused-function -Wno-deprecated-register
 -Wno-inconsistent-missing-override -Wno-deprecated-register)
#add_definitions(-g -rdynamic)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CSAPEX_BOOT_PLUGIN_DIR $ENV{HOME}/.csapex/boot)


## Enforce that we use C++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()
