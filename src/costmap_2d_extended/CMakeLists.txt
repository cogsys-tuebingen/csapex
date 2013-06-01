cmake_minimum_required(VERSION 2.8.3)
project(costmap_2d_extended)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS genmsg roscpp dynamic_reconfigure std_msgs geometry_msgs pcl pcl_ros voxel_grid laser_geometry)

## Find message files
file(GLOB messages messages RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/msg/"
    "${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg")

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  ${messages}
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

generate_dynamic_reconfigure_options(cfg/Costmap2D.cfg)

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES costmap_2d_extended
   CATKIN_DEPENDS pcl pcl_ros voxel_grid laser_geometry
#  DEPENDS system_lib
)


include_directories(include
	${catkin_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}/cfg/cpp/
)

add_library(costmap_2d_extended src/costmap_2d.cpp src/observation_buffer.cpp src/costmap_2d_ros.cpp src/costmap_2d_publisher.cpp src/voxel_costmap_2d.cpp)
target_link_libraries(costmap_2d_extended ${catkin_LIBRARIES})
add_dependencies(costmap_2d_extended costmap_2d_extended_gencfg) 

add_executable(costmap_2d_extended_node src/costmap_2d_node.cpp)
target_link_libraries(costmap_2d_extended_node costmap_2d_extended ${catkin_LIBRARIES})

