/*
 * pcl_filter.cpp
 *
 *  Created on: 6 19, 2013
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
#include <pcl_demo/pcl_filterConfig.h>

class PclFilter
{
public:
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

    PclFilter()
        : nh_("~")
    {
        std::string topic = "/camera/depth_registered/points";

        nh_.param("cloud_topic", topic, topic);

        sub_ = nh_.subscribe<PointCloud>(topic, 1, boost::bind(&PclFilter::callback, this, _1));
        pub_ = nh_.advertise<PointCloud>(topic + "/filtered", 1, true);

        f = boost::bind(&PclFilter::dynamicReconfigureCallback, this, _1, _2);
        server.setCallback(f);
    }

    void dynamicReconfigureCallback(pcl_demo::pcl_filterConfig &config, uint32_t level) {
        leaf_ = config.leaf_size;
    }

    void callback(const PointCloud::ConstPtr& cloud)
    {
        // All the objects needed
        pcl::PassThrough<PointT> pass;
        pcl::VoxelGrid<PointT> voxel;

        // Datasets
        pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);

        voxel.setInputCloud(cloud);
        voxel.setLeafSize(leaf_, leaf_, leaf_);
        voxel.filter(*cloud_filtered);

        // Build a passthrough filter to remove spurious NaNs
        pass.setInputCloud (cloud_filtered);
        pass.setFilterFieldName ("x");
        pass.setFilterLimits (-0.4, 0.4);
        pass.filter (*cloud_filtered2);
        pass.setInputCloud (cloud_filtered2);
        pass.setFilterFieldName ("y");
        pass.setFilterLimits (-0.2, 3.8);
        pass.filter (*cloud_filtered);


        pub_.publish(cloud_filtered);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;

    dynamic_reconfigure::Server<pcl_demo::pcl_filterConfig> server;
    dynamic_reconfigure::Server<pcl_demo::pcl_filterConfig>::CallbackType f;

    double leaf_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_filter_node");
    ROS_INFO_STREAM("starting PCL filter node");

    PclFilter filter;

    ros::WallRate rate(30);

    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
}

