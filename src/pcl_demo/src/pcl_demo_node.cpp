/*
 * pcl_demo_node.cpp
 *
 *  Created on: 4 12, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

/// SYSTEM
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <visualization_msgs/Marker.h>

#include <dynamic_reconfigure/server.h>
#include <pcl_demo/pcl_demoConfig.h>
#include <tf/transform_listener.h>
#include <tf/exceptions.h>

class PclDemo
{
public:
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

    PclDemo()
        : nh_("~")
    {
        std::string topic = "/camera/depth_registered/points/filtered";

        nh_.param("cloud_topic", topic, topic);

        target_frame = "/base_link";
        nh_.param("target_frame", target_frame, target_frame);

        memory_frame = "/odom";
        nh_.param("memory_frame", memory_frame, memory_frame);


        sub_ = nh_.subscribe<PointCloud>(topic, 1, boost::bind(&PclDemo::callback, this, _1));
        pub_plane_ = nh_.advertise<PointCloud>("plane", 1, true);
        pub_sphere_ = nh_.advertise<PointCloud>("sphere", 1, true);
        pub_balls_ = nh_.advertise<PointCloud>("balls", 1, true);
        pub_goal_pos_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, true);

        has_ball = false;
        max_wait_time = ros::Duration(1);
        last = ros::Time::now();

        init_transformation = true;

        cloud.reset(new PointCloud);

        f = boost::bind(&PclDemo::dynamicReconfigureCallback, this, _1, _2);
        server.setCallback(f);
    }

    void dynamicReconfigureCallback(pcl_demo::pcl_demoConfig &config, uint32_t level) {
        ransac_plane_ = config.ransac_plane;
        ransac_sphere_ = config.ransac_sphere;

        sphere_r_min_ = config.sphere_radius_min;
        sphere_r_max_ = config.sphere_radius_max;

        iterations_plane_ = config.iterations_plane;
        iterations_sphere_ = config.iterations_sphere;

        sphere_min_points = config.sphere_min_points;

        sphere_distance_threshold = config.sphere_distance_threshold;
        sphere_normal_distance_weight = config.sphere_normal_distance_weight;

        planes_ = config.planes;
        spheres_ = config.spheres;


        segmenter_plane.setOptimizeCoefficients (true);
        segmenter_plane.setModelType (pcl::SACMODEL_NORMAL_PLANE);
        segmenter_plane.setNormalDistanceWeight (0.1);
        segmenter_plane.setMethodType (ransac_plane_);
        segmenter_plane.setMaxIterations (iterations_plane_);
        segmenter_plane.setDistanceThreshold (0.025);

        segmenter_sphere.setOptimizeCoefficients (true);
        segmenter_sphere.setModelType (pcl::SACMODEL_SPHERE);
        segmenter_sphere.setMethodType (ransac_sphere_);
        segmenter_sphere.setNormalDistanceWeight (sphere_normal_distance_weight);
        segmenter_sphere.setMaxIterations (iterations_sphere_);
        segmenter_sphere.setDistanceThreshold (sphere_distance_threshold);
        segmenter_sphere.setRadiusLimits (sphere_r_min_, sphere_r_max_);
    }

    void estimateNormals()
    {
        pcl::NormalEstimation<PointT, pcl::Normal> normal_estimation;
        pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);

        normal_estimation.setSearchMethod (tree);
        normal_estimation.setInputCloud (cloud);
        normal_estimation.setKSearch (50);
        normal_estimation.compute (*normals);
    }

    bool findAndExtractPlane()
    {
        pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
        PointCloud::Ptr cloud_plane (new PointCloud);

        cloud_plane->header = cloud->header;

        for(int i=0; i < std::max(1, planes_); ++i){
            segmenter_plane.setInputCloud (cloud);
            segmenter_plane.setInputNormals (normals);
            segmenter_plane.segment (*inliers_plane, *coefficients_plane);

            if(inliers_plane->indices.size() < 100) {
                continue;
            }

            extract_points.setInputCloud (cloud);
            extract_points.setIndices (inliers_plane);
            extract_points.setNegative (false);

            PointCloud::Ptr extraced_plane (new PointCloud);
            extract_points.filter (*extraced_plane);

            *cloud_plane = *extraced_plane;

            extract_points.setNegative (true);
            extract_points.filter (*cloud);

            extract_normals.setNegative (true);
            extract_normals.setInputCloud (normals);
            extract_normals.setIndices (inliers_plane);
            extract_normals.filter (*normals);

            // update plane model
            plane[0] = coefficients_plane->values[0];
            plane[1] = coefficients_plane->values[1];
            plane[2] = coefficients_plane->values[2];
            plane[3] = coefficients_plane->values[3];

            pub_plane_.publish(cloud_plane);

            return true;
        }

        return false;
    }

    void findBalls()
    {
        pcl::PointIndices::Ptr inliers_sphere (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients_sphere (new pcl::ModelCoefficients);
        PointCloud::Ptr cloud_sphere (new PointCloud);

        cloud_sphere->header = cloud->header;
        balls->header = cloud->header;

        for(int i=0; i < spheres_; ++i){
            segmenter_sphere.setInputCloud (cloud);
            segmenter_sphere.setInputNormals (normals);

            segmenter_sphere.segment (*inliers_sphere, *coefficients_sphere);

            if(inliers_sphere->indices.size() < sphere_min_points) {
                continue;
            }

            pcl::PointXYZ sphere_center;
            sphere_center.x = coefficients_sphere->values[0];
            sphere_center.y = coefficients_sphere->values[1];
            sphere_center.z = coefficients_sphere->values[2];

            double dist = pcl::pointToPlaneDistanceSigned(sphere_center, plane);

            if(dist < - 3 * sphere_r_max_ || dist > 3 * sphere_r_max_) {
                std::cout << "skipping ball with distance " << dist << std::endl;
                continue;
            }

            extract_points.setInputCloud (cloud);
            extract_points.setIndices (inliers_sphere);
            extract_points.setNegative (false);

            PointCloud::Ptr extracted_sphere_points (new PointCloud);
            extract_points.filter (*extracted_sphere_points);

            *cloud_sphere += *extracted_sphere_points;

            extractBall(sphere_center, extracted_sphere_points);

            extract_points.setNegative (true);
            extract_points.filter (*cloud);

            extract_normals.setNegative (true);
            extract_normals.setInputCloud (normals);
            extract_normals.setIndices (inliers_sphere);
            extract_normals.filter (*normals);
        }

        pub_balls_.publish(balls);
        pub_sphere_.publish(cloud_sphere);
    }

    void extractBall(pcl::PointXYZ center, PointCloud::Ptr points)
    {
        PointT ball;
        ball.x = center.x;
        ball.y = center.y;
        ball.z = center.z;
        ball.a = 255;

        int r = 0;
        int g = 0;
        int b = 0;

        double c = 0;
        BOOST_FOREACH(PointT& center, points->points) {
            r += center.r;
            g += center.g;
            b += center.b;

            ++c;
        }

        ball.r = r / c;
        ball.g = g / c;
        ball.b = b / c;

        balls->points.push_back(ball);
    }

    void processBalls()
    {
        BOOST_FOREACH(PointT& ball, balls->points) {
            ROS_INFO_STREAM("ball: rgb: " << (int) ball.r << " / " << (int) ball.g << " / " << (int) ball.b);
            ROS_INFO_STREAM("ball: " << (ball.g > ball.b * 1.2) << " && " << (ball.g > ball.r * 1.2));
            if(isMagenta(ball)) {
                has_ball = true;
                last_ball_odom = transformPoint(ball, memory_frame, balls->header.frame_id);
                last_ball_time = ros::Time::now();
                return;
            }
        }

    }

    bool isBlue(PointT& ball)
    {
        return ball.b > ball.g * 1.2 && ball.b > ball.r * 1.2;
    }

    bool isMagenta(PointT& ball)
    {
        return ball.r > ball.g * 1.2 && ball.r > ball.b * 1.2;
    }

    bool isGreen(PointT& ball)
    {
        return ball.g > ball.b * 1.2 && ball.g > ball.r * 1.2;
    }

    PointT transformPoint(const PointT& pt,const std::string& target_frame, const std::string& source_frame, tf::Quaternion* quat = NULL)
    {
        tf::StampedTransform transformation;
        try {
            tfl.lookupTransform(target_frame, source_frame, ros::Time(0), transformation);
        } catch(tf::TransformException& e) {
            ROS_WARN_STREAM(e.what());
            tfl.clear();
            return PointT();
        }

        tf::Vector3 ball_pt(pt.x, pt.y, pt.z);
        tf::Vector3 ball_pt_target_frame = transformation (ball_pt);

        PointT result;
        result.x = ball_pt_target_frame.x();
        result.y = ball_pt_target_frame.y();
        result.z = ball_pt_target_frame.z();

        if(quat) {
            *quat = tf::createQuaternionFromYaw(std::atan2(ball_pt_target_frame.y(), ball_pt_target_frame.x()));
        }

        return result;
    }

    void sendBallPoseAsGoalPose(PointT& ball)
    {
        tf::Quaternion quat;
        PointT ball_target_frame = transformPoint(ball, target_frame, memory_frame, &quat);

        geometry_msgs::PoseStamped goal;
        goal.pose.position.x = ball_target_frame.x;
        goal.pose.position.y = ball_target_frame.y;
        goal.pose.position.z = ball_target_frame.z;

        tf::quaternionTFToMsg(quat, goal.pose.orientation);

        goal.header.frame_id = target_frame;

        pub_goal_pos_.publish(goal);
    }

    void reset(const PointCloud::ConstPtr& input)
    {
        normals.reset(new pcl::PointCloud<pcl::Normal>);
        balls.reset (new PointCloud);
        *cloud = *input;
    }

    void callback(const PointCloud::ConstPtr& input)
    {
        if(init_transformation) {
            try {
                init_transformation = tfl.waitForTransform(input->header.frame_id, target_frame, ros::Time(0), ros::Duration(0.5));
            } catch(tf::ConnectivityException& e) {
                ROS_WARN_STREAM(e.what());
            }
        }

        reset(input);

        estimateNormals();

        if(!findAndExtractPlane()) {
            ROS_DEBUG_STREAM("cannot find the plane");
        }

        findBalls();

        processBalls();
    }

    void tick()
    {
        ros::Time now = ros::Time::now();

        if(now < last) {
            tfl.clear();
            init_transformation = true;
        }

        last = now;

        if(has_ball && ros::Time::now() < last_ball_time + max_wait_time) {
            sendBallPoseAsGoalPose(last_ball_odom);

        } else {
            geometry_msgs::PoseStamped goal;
            goal.pose.position.x = 0.5;
            goal.pose.position.y = 0;
            goal.pose.position.z = 0;

            goal.pose.orientation = tf::createQuaternionMsgFromYaw(0);

            goal.header.frame_id = target_frame;

            pub_goal_pos_.publish(goal);
        }

    }

private:
    ros::NodeHandle nh_;
    ros::Time last;

    ros::Subscriber sub_;
    ros::Publisher pub_plane_;
    ros::Publisher pub_sphere_;
    ros::Publisher pub_balls_;
    ros::Publisher pub_goal_pos_;

    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::ExtractIndices<PointT> extract_points;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> segmenter_plane;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> segmenter_sphere;

    PointCloud::Ptr cloud;
    pcl::PointCloud<pcl::Normal>::Ptr normals;
    PointCloud::Ptr balls;

    Eigen::Vector4f plane;

    dynamic_reconfigure::Server<pcl_demo::pcl_demoConfig> server;
    dynamic_reconfigure::Server<pcl_demo::pcl_demoConfig>::CallbackType f;

    tf::TransformListener tfl;

    std::string memory_frame;
    std::string target_frame;

    bool init_transformation;

    bool has_ball;
    PointT last_ball_odom;
    ros::Time last_ball_time;
    ros::Duration max_wait_time;

    int ransac_plane_;
    int ransac_sphere_;

    int planes_;
    int spheres_;

    int iterations_plane_;
    int iterations_sphere_;

    int sphere_min_points;

    double sphere_normal_distance_weight;
    double sphere_distance_threshold;

    double sphere_r_min_;
    double sphere_r_max_;
    double leaf_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_demo_node");
    ROS_INFO_STREAM("starting PCL demo node");

    PclDemo demo;

    ros::WallRate rate(30);  

    while(ros::ok()) {
        demo.tick();
        ros::spinOnce();
        rate.sleep();
    }
}
