#ifndef SIMPLE_TRACKER_FRAMEWORK_H
#define SIMPLE_TRACKER_FRAMEWORK_H

/// SYSTEM
#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl_demo/pcl_demoConfig.h>

#include <tf/transform_listener.h>

class SimpleTracker
{
public:
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

public:
    SimpleTracker();

    void tick();

    void callback(const PointCloud::ConstPtr& input);
    void dynamicReconfigureCallback(pcl_demo::pcl_demoConfig &config, uint32_t level);

private:
    void estimateNormals();
    bool findAndExtractPlane();
    void findAndExtractBalls();
    void extractBall(pcl::PointXYZ center, PointCloud::Ptr points);
    void processBalls();

    PointT transformPoint(const PointT& pt,const std::string& target_frame_, const std::string& source_frame, tf::Quaternion* quat = NULL);

    void sendBallPoseAsGoalPose(PointT& ball);
    void sendCarrotPose();

    void init(const PointCloud::ConstPtr& input);
    void initializeSegmentations();

private:
    /// THESE MEMBERS ARE GIVEN
    ros::NodeHandle nh_;

    std::string memory_frame_;
    std::string target_frame_;

    ros::Subscriber sub_cloud_;
    ros::Publisher pub_marker_;
    ros::Publisher pub_plane_;
    ros::Publisher pub_sphere_;
    ros::Publisher pub_balls_;
    ros::Publisher pub_goal_pos_;

    pcl::ExtractIndices<pcl::Normal> extract_normals_;
    pcl::ExtractIndices<PointT> extract_points_;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> segmenter_plane_;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> segmenter_sphere_;

    PointCloud::Ptr cloud_;
    pcl::PointCloud<pcl::Normal>::Ptr normals_;
    PointCloud::Ptr balls_;

    Eigen::Vector4f plane_model_;

    dynamic_reconfigure::Server<pcl_demo::pcl_demoConfig> server_;
    dynamic_reconfigure::Server<pcl_demo::pcl_demoConfig>::CallbackType f_;

    ros::Time last_tf_check_;
    tf::TransformListener tfl_;
    bool init_transformation_;

    /// YOUR MEMBERS HERE

    /// DYNAMICALLY RECONFIGURED VARIABLES:
    int ransac_plane_;
    int ransac_sphere_;

    int no_of_planes_to_extract_;
    int no_of_spheres_to_extract_;

    int iterations_plane_;
    int iterations_sphere_;

    int sphere_min_points_;

    double sphere_normal_distance_weight_;
    double sphere_distance_threshold_;

    double sphere_r_min_;
    double sphere_r_max_;
};

#endif // SIMPLE_TRACKER_FRAMEWORK_H
