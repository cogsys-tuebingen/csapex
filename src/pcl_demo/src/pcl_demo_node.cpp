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

#include <visualization_msgs/Marker.h>

#include <dynamic_reconfigure/server.h>
#include <pcl_demo/pcl_demoConfig.h>

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

        sub_ = nh_.subscribe<PointCloud>(topic, 1, boost::bind(&PclDemo::callback, this, _1));
        pub_plane_ = nh_.advertise<PointCloud>("plane", 1, true);
        pub_sphere_ = nh_.advertise<PointCloud>("sphere", 1, true);
        pub_marker_ = nh_.advertise<visualization_msgs::Marker>("/visualization_marker", 1, true);

        id = 0;

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

        planes_ = config.planes;
        spheres_ = config.spheres;

        std::cout << "reconfigure: " << spheres_ << std::endl;
    }

    void callback(const PointCloud::ConstPtr& cloud)
    {
        // All the objects needed
        pcl::NormalEstimation<PointT, pcl::Normal> ne;
        pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
        pcl::ExtractIndices<PointT> extract;
        pcl::ExtractIndices<pcl::Normal> extract_normals;
        pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

        // Datasets
        pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
        pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_sphere (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_sphere (new pcl::PointIndices);

        *cloud_filtered = *cloud;

        // Estimate point normals
        ne.setSearchMethod (tree);
        ne.setInputCloud (cloud_filtered);
        ne.setKSearch (50);
        ne.compute (*cloud_normals);

        pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
        cloud_plane->header = cloud->header;

        // Create the segmentation object for the planar model and set all the parameters
        for(int i=0; i < planes_; ++i){
            seg.setOptimizeCoefficients (true);
            seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
            seg.setNormalDistanceWeight (0.1);
            seg.setMethodType (ransac_plane_);
            seg.setMaxIterations (iterations_plane_);
            seg.setDistanceThreshold (0.05);
            seg.setInputCloud (cloud_filtered);
            seg.setInputNormals (cloud_normals);
            // Obtain the plane inliers and coefficients
            seg.segment (*inliers_plane, *coefficients_plane);
            //              std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

            // Extract the planar inliers from the input cloud
            extract.setInputCloud (cloud_filtered);
            extract.setIndices (inliers_plane);
            extract.setNegative (false);

            if(inliers_plane->indices.size() < 1000) {
                std::cerr << "no plane" << std::endl;
                break;
            }
            std::cerr << "Plane" << std::endl;

            pcl::PointCloud<PointT>::Ptr tmp (new pcl::PointCloud<PointT> ());
            extract.filter (*tmp);

            *cloud_plane += *tmp;

            // Remove the planar inliers, extract the rest
            extract.setNegative (true);
            extract.filter (*cloud_filtered2);
            extract_normals.setNegative (true);
            extract_normals.setInputCloud (cloud_normals);
            extract_normals.setIndices (inliers_plane);
            extract_normals.filter (*cloud_normals2);

            cloud_filtered = cloud_filtered2;
            cloud_normals = cloud_normals2;
        }


        // Publish the planar inliers
        pub_plane_.publish(cloud_plane);
        std::cout << "publish plane (size=" << cloud_plane->points.size() << ")" << std::endl;

        pcl::PointCloud<PointT>::Ptr cloud_sphere (new pcl::PointCloud<PointT> ());
        cloud_sphere->header = cloud->header;

        visualization_msgs::Marker balls;

        // Create the segmentation object for cylinder segmentation and set all the parameters
        for(int i=0; i < spheres_; ++i){
            seg.setOptimizeCoefficients (true);
            seg.setModelType (pcl::SACMODEL_SPHERE);
            seg.setMethodType (ransac_sphere_);
            seg.setNormalDistanceWeight (0.1);
            seg.setMaxIterations (iterations_sphere_);
            seg.setDistanceThreshold (0.03);
            seg.setRadiusLimits (sphere_r_min_, sphere_r_max_);
            seg.setInputCloud (cloud_filtered);
            seg.setInputNormals (cloud_normals);

            std::cerr << "Sphere coefficients " << i << std::endl;
            // Obtain the sphere inliers and coefficients
            seg.segment (*inliers_sphere, *coefficients_sphere);

            if(inliers_sphere->indices.size() < 4) {
                break;
            }
            std::cerr << "Sphere coefficients (" << i << "): " << *coefficients_sphere << std::endl;

            geometry_msgs::Point pt;
            pt.x = coefficients_sphere->values[0];
            pt.y = coefficients_sphere->values[1];
            pt.z = coefficients_sphere->values[2];
            balls.points.push_back(pt);

            // Write the cylinder inliers to disk
            extract.setInputCloud (cloud_filtered);
            extract.setIndices (inliers_sphere);
            extract.setNegative (false);

            pcl::PointCloud<PointT>::Ptr tmp (new pcl::PointCloud<PointT> ());
            extract.filter (*tmp);

            *cloud_sphere += *tmp;

            // Remove the sphere inliers, extract the rest
            extract.setNegative (true);
            extract.filter (*cloud_filtered2);
            extract_normals.setNegative (true);
            extract_normals.setInputCloud (cloud_normals);
            extract_normals.setIndices (inliers_sphere);
            extract_normals.filter (*cloud_normals2);

            cloud_filtered = cloud_filtered2;
            cloud_normals = cloud_normals2;


            ros::spinOnce();
        }

        pub_sphere_.publish(cloud_sphere);

        if (balls.points.size()) {
            std::cerr << "Can't find the spherical component." << std::endl;
        }

        balls.action = visualization_msgs::Marker::ADD;
        balls.type = visualization_msgs::Marker::SPHERE_LIST;
        balls.color.a = 1.0;
        balls.color.r = 1.0;
        balls.color.g = 1.0;
        balls.color.b = 1.0;

        balls.lifetime = ros::Duration(1.0);

        double r = 0.3;
        balls.scale.x = r;
        balls.scale.y = r;
        balls.scale.z = r;

        balls.header = cloud->header;
        balls.id = id++;
        balls.ns = "balls";

        pub_marker_.publish(balls);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_plane_;
    ros::Publisher pub_sphere_;
    ros::Publisher pub_marker_;

    dynamic_reconfigure::Server<pcl_demo::pcl_demoConfig> server;
    dynamic_reconfigure::Server<pcl_demo::pcl_demoConfig>::CallbackType f;

    int ransac_plane_;
    int ransac_sphere_;

    int planes_;
    int spheres_;

    int iterations_plane_;
    int iterations_sphere_;

    double sphere_r_min_;
    double sphere_r_max_;
    double leaf_;

    int id;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_demo_node");
    ROS_INFO_STREAM("starting PCL demo node");

    PclDemo demo;

    ros::WallRate rate(30);

    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
}
