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

#include <visualization_msgs/Marker.h>

class PclDemo
{
public:
    typedef pcl::PointXYZI PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

    PclDemo()
        : nh_("~")
    {
        sub_ = nh_.subscribe<PointCloud>("/fotonic_e70_node/cloud", 1, boost::bind(&PclDemo::callback, this, _1));
        pub_plane_ = nh_.advertise<PointCloud>("plane", 1, true);
        pub_sphere_ = nh_.advertise<PointCloud>("sphere", 1, true);
        pub_marker_ = nh_.advertise<visualization_msgs::Marker>("/visualization_marker", 1, true);

        id = 0;
    }

    void callback(const PointCloud::ConstPtr& cloud)
    {
        // All the objects needed
        pcl::PCDReader reader;
        pcl::PassThrough<PointT> pass;
        pcl::NormalEstimation<PointT, pcl::Normal> ne;
        pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
        pcl::PCDWriter writer;
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

        // Build a passthrough filter to remove spurious NaNs
        pass.setInputCloud (cloud);
        //        pass.setFilterFieldName ("z");
        //        pass.setFilterLimits (-4, 4);
        pass.filter (*cloud_filtered);

        // Estimate point normals
        ne.setSearchMethod (tree);
        ne.setInputCloud (cloud_filtered);
        ne.setKSearch (50);
        ne.compute (*cloud_normals);

        pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
        cloud_plane->header = cloud->header;

        // Create the segmentation object for the planar model and set all the parameters
        for(int i=0; i < 15; ++i){
            seg.setOptimizeCoefficients (true);
            seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
            seg.setNormalDistanceWeight (0.1);
            seg.setMethodType (pcl::SAC_RANSAC);
            seg.setMaxIterations (100);
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
                break;
            }

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

        pcl::PointCloud<PointT>::Ptr cloud_sphere (new pcl::PointCloud<PointT> ());
        cloud_sphere->header = cloud->header;

        visualization_msgs::Marker balls;

        // Create the segmentation object for cylinder segmentation and set all the parameters
        std::cerr << "Sphere coefficients" << std::endl;
        for(int i=0; i < 4; ++i){
            seg.setOptimizeCoefficients (true);
            seg.setModelType (pcl::SACMODEL_SPHERE);
            seg.setMethodType (pcl::SAC_RANSAC);
            seg.setNormalDistanceWeight (0.1);
            seg.setMaxIterations (20000);
            seg.setDistanceThreshold (0.03);
            seg.setRadiusLimits (0.08, 0.17);
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
