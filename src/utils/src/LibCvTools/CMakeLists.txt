# Utils/LibCvTools/CMakeLists.txt

# Load OpenCV cmake module, if wasn't looked for or found yet
find_package(OpenCV REQUIRED)
set(CMAKE_BUILD_TYPE Debug)

#set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_CXX_FLAGS "-O3")

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
    messages.hpp
)

target_link_libraries(LibCvTools
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    yaml-cpp
)

####### Tools #######
add_executable(undistort
    undistort.cpp)

target_link_libraries(undistort
    ${OpenCV_LIBS}
    LibCvTools)

add_executable(scale
    scale.cpp)

target_link_libraries(scale
    ${OpenCV_LIBS})

add_executable(equalize
    equalize.cpp)

target_link_libraries(equalize
    ${OpenCV_LIBS})

add_executable(gamma_correct
    gamma.cpp)

target_link_libraries(gamma_correct
    ${OpenCV_LIBS})

add_executable(y_extract
    y_extract.cpp)

target_link_libraries(y_extract
    ${OpenCV_LIBS})

add_executable(terramatRenderer
    terra_mat_renderer.cpp)

target_link_libraries(terramatRenderer
    ${OpenCV_LIBS}
    LibCvTools)

####### Tests #######
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
