cmake_minimum_required(VERSION 2.8.3)
project(csapex_point_cloud)

ADD_DEFINITIONS(-DQT_NO_KEYWORDS)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS csapex csapex_core_plugins csapex_transform csapex_vision pcl_ros)

csapex_moc_all(QT_MOC src/*.h include/csapex_point_cloud/*.h)

set(QT ${QT_SOURCES} ${QT_MOC})

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES csapex_point_cloud
  CATKIN_DEPENDS csapex pcl_ros
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(csapex_point_cloud
    src/point_cloud_message.cpp
    src/model_message.cpp
    ${QT}
)
target_link_libraries(csapex_point_cloud
    yaml-cpp ${QT_LIBRARIES} ${catkin_LIBRARIES})

add_library(csapex_point_cloud_plugin
    src/register_plugin.cpp
    src/point_count.cpp
    src/transform_cloud.cpp
    src/voxel_grid.cpp
    src/extract_timestamp.cpp
    src/set_timestamp.cpp
    src/crop_box.cpp
    src/passthrough.cpp
    src/pointcloud_to_depthimage.cpp
    src/pointcloud_to_intensityimage.cpp
    src/pointcloud_to_pointmatrix.cpp
    src/pointmatrix_to_pointcloud.cpp
    src/pointcloud_generator.cpp
    src/image_to_point_cloud.cpp
    src/statistical_outlier_removal.cpp
    src/radius_outlier_removal.cpp
    src/conditional_outlier_removal.cpp
    src/sac_fit.cpp
    src/model_to_marker.cpp
    src/color_pointcloud.cpp
    src/label_pointcloud.cpp
    src/cluster_pointcloud.cpp
    src/labeled_cloud_to_indices.cpp
    src/split_clustered_cloud.cpp
    ${QT}
)
target_link_libraries(csapex_point_cloud_plugin csapex_core_plugins
    yaml-cpp ${QT_LIBRARIES} ${catkin_LIBRARIES}
    csapex_point_cloud)

