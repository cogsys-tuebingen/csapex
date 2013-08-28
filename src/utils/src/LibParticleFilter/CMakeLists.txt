set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_executable(test
test.cpp
pfilter.cpp
)

add_library(LibParticleFilter
    pfilter.cpp)

install(TARGETS LibParticleFilter LIBRARY DESTINATION lib)

# INSTALL INCLUDE HEADERS
include(../cmake/install_symlink.cmake)
