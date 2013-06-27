# Utils/LibCvTools/CMakeLists.txt

# Load OpenCV cmake module, if wasn't looked for or found yet
find_package(OpenCV REQUIRED)

add_library (LibCvTools SHARED
    undistorter.cpp
    perspective_transform.cpp
    grid.hpp
    grid_attributes.hpp
    histogram.hpp
    quad_tree.hpp
    decomposition_quadtree.cpp
    decomposition_classifiers.hpp
)

target_link_libraries(LibCvTools
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
)

install(TARGETS LibCvTools LIBRARY DESTINATION lib)

# INSTALL INCLUDE HEADERS
include(../cmake/install_symlink.cmake)
