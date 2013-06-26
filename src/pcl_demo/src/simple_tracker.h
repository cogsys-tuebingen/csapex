#ifndef SIMPLE_TRACKER_H
#define SIMPLE_TRACKER_H

/// SYSTEM
#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
//#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
//#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl_demo/pcl_demoConfig.h>

#include <tf/transform_listener.h>
//#include <tf/exceptions.h>

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

    bool isBlue(PointT& ball);
    bool isMagenta(PointT& ball);
    bool isGreen(PointT& ball);

    PointT transformPoint(const PointT& pt,const std::string& target_frame_, const std::string& source_frame, tf::Quaternion* quat = NULL);

    void sendBallPoseAsGoalPose(PointT& ball);
    void sendCarrotPose();

    void init(const PointCloud::ConstPtr& input);
    void initializeSegmentations();

private:
    ros::NodeHandle nh_;
    ros::Time last_tf_check_;

    std::string memory_frame_;
    std::string target_frame_;

    ros::Subscriber sub_cloud_;
    ros::Publisher pub_marker_;
    ros::Publisher pub_plane_;
    ros::Publisher pub_sphere_;
    ros::Publisher pub_balls_;
    ros::Publisher pub_goal_pos_;

    /// YOUR MEMBERS HERE
    // <STUDENTS SHOULD PRODUCE>
    bool has_ball;
    PointT last_ball_odom;
    ros::Time last_ball_time;
    ros::Duration max_wait_time;
    // </STUDENTS SHOULD PRODUCE>

    /// THESE MEMBERS ARE GIVEN
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

    tf::TransformListener tfl_;

    bool init_transformation_;

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

#endif // SIMPLE_TRACKER_H
