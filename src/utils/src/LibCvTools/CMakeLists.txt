# Utils/LibCvTools/CMakeLists.txt

# Load OpenCV cmake module, if wasn't looked for or found yet
find_package(OpenCV REQUIRED)

SET(CMAKE_BUILD_TYPE Debug)

add_library (LibCvTools SHARED
    undistorter.cpp
    perspective_transform.cpp
    terra_decomposition_quadtree.cpp
    grid.hpp
    grid_attributes.hpp
    grid_compute.hpp
    histogram.hpp
    quad_tree.hpp
    decomposition_quadtree.cpp
    decomposition_classifiers.hpp
    extractor.cpp
    randomforest.cpp
    roi.hpp
    local_patterns.hpp
)

target_link_libraries(LibCvTools
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
)

install(TARGETS LibCvTools LIBRARY DESTINATION lib)

# INSTALL INCLUDE HEADERS
include(../cmake/install_symlink.cmake)
