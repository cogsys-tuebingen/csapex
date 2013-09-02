# Utils/LibCvTools/CMakeLists.txt

# Load OpenCV cmake module, if wasn't looked for or found yet
find_package(OpenCV REQUIRED)

SET(CMAKE_BUILD_TYPE Debug)

add_library (LibCvTools SHARED
    undistorter.cpp
    perspective_transform.cpp
    grid.hpp
    grid_attributes.hpp
    grid_compute.hpp
    histogram.hpp
    quad_tree.hpp
    decomposition_quadtree.cpp
    decomposition_classifiers.hpp
    randomforest.cpp
    roi.hpp
    local_patterns.hpp
    extractor.cpp
    pattern_extractor.cpp
    feature_extractor.cpp
    extractor_params.cpp
    terra_mat.cpp
    terra_mat_generation.hpp
    terra_decomposition_quadtree.cpp
    terra_decomposition_classifier.cpp
    math.hpp
)

target_link_libraries(LibCvTools
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    yaml-cpp
)

# Rotated template test
add_executable(rotated_template
    rotated_template.cpp
    terra_mat.cpp)

target_link_libraries(rotated_template
    LibParticleFilter
    ${OpenCV_LIBS})

# terra map test
add_executable(test_terra_map
    test_terra_mat.cpp)

target_link_libraries(test_terra_map
    LibCvTools
    ${OpenCV_LIBS})

# random forest loading test
add_executable(randomforest_loading_test
    randomforest_loading_test.cpp)

target_link_libraries(randomforest_loading_test
    LibCvTools
    ${OpenCV_LIBS})

install(TARGETS LibCvTools LIBRARY DESTINATION lib)

# INSTALL INCLUDE HEADERS
include(../cmake/install_symlink.cmake)
