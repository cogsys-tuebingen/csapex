/// HEADER
#include "register_plugin.h"

/// COMPONENT
#include <csapex_point_cloud/point_cloud_message.h>

/// PROJECT
#include <csapex/manager/connection_type_manager.h>
#include <csapex/model/tag.h>
#include <csapex_core_plugins/ros_message_conversion.h>

/// SYSTEM
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

PLUGINLIB_EXPORT_CLASS(csapex::RegisterPointCloudPlugin, csapex::CorePlugin)

using namespace csapex;


struct Sensor2Cloud
{
    struct Export : public boost::static_visitor<void> {
        Export(sensor_msgs::PointCloud2::Ptr &out)
            : out_(out)
        {}

        template <typename T>
        void operator () (T cloud) const
        {
            pcl::toROSMsg(*cloud, *out_);
        }

        sensor_msgs::PointCloud2::Ptr &out_;
    };

    static void ros2apex(const sensor_msgs::PointCloud2::ConstPtr &ros_msg, typename connection_types::PointCloudMessage::Ptr& out) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*ros_msg, *cloud);
        out->value = cloud;
    }
    static void apex2ros(const typename connection_types::PointCloudMessage::Ptr& apex_msg, sensor_msgs::PointCloud2::Ptr &out) {
        boost::apply_visitor (Export(out), apex_msg->value);
    }
};



RegisterPointCloudPlugin::RegisterPointCloudPlugin()
{
}

void RegisterPointCloudPlugin::init()
{
    Tag::createIfNotExists("PointCloud");

    ConnectionTypeManager::registerMessage<connection_types::PointCloudMessage>();

    RosMessageConversion::registerConversion<sensor_msgs::PointCloud2, connection_types::PointCloudMessage, Sensor2Cloud >();
}
