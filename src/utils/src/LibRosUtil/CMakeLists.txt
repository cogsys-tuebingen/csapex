
add_library(LibRosUtil
    Costmap2dWrapper.cpp
    OccupancyGridWrapper.cpp
    RosLaserScan.cpp
)

target_link_libraries(LibRosUtil LibPath ${catkin_LIBRARIES})




# INSTALL INCLUDE HEADERS

include(../cmake/install_symlink.cmake)
