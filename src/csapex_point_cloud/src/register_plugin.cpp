/// HEADER
#include "register_plugin.h"

/// COMPONENT
#include <csapex_point_cloud/point_cloud_message.h>

/// PROJECT
#include <csapex/manager/connection_type_manager.h>
#include <csapex/model/tag.h>
#include <csapex_core_plugins/ros_message_conversion.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/impl/io.hpp>
#include <boost/mpl/for_each.hpp>

CSAPEX_REGISTER_CLASS(csapex::RegisterPointCloudPlugin, csapex::CorePlugin)

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

    struct try_convert
    {
        try_convert(const sensor_msgs::PointCloud2::ConstPtr &ros_msg, typename connection_types::PointCloudMessage::Ptr& out, bool& success)
            : ros_msg_(ros_msg), out_(out), success_(success)
        {
            success_ = false;
        }

        template<typename PointT>
        void operator()(PointT& pt)
        {
            if(success_) {
                return;
            }

            std::vector<pcl::PCLPointField> fields;
            pcl::for_each_type<typename pcl::traits::fieldList<PointT>::type>(pcl::detail::FieldAdder<PointT>(fields));

            const sensor_msgs::PointCloud2::_fields_type& available_fields = ros_msg_->fields;


            if(fields.size() != available_fields.size()) {
                return;
            }

            for(size_t d = 0; d < fields.size (); ++d) {
                bool found = false;
                for(size_t f = 0; f < available_fields.size() && !found; ++f) {
                    if(fields[d].name == available_fields[f].name) {
                        found = true;
                    }
                }

                if(!found) {
                    return;
                }
            }

            success_ = true;
            typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
            pcl::fromROSMsg(*ros_msg_, *cloud);
            out_->value = cloud;
            out_->type = connection_types::traits::name<PointT>();
        }

        const sensor_msgs::PointCloud2::ConstPtr &ros_msg_;
        typename connection_types::PointCloudMessage::Ptr& out_;

        bool& success_;
    };

    static void ros2apex(const sensor_msgs::PointCloud2::ConstPtr &ros_msg, typename connection_types::PointCloudMessage::Ptr& out) {
        bool success;
        try_convert converter(ros_msg, out, success);
        boost::mpl::for_each<connection_types::PointCloudPointTypes>( converter );

        if(!converter.success_) {
            std::cerr << "cannot convert message, type is not known" << std::endl;
        }
    }
    static void apex2ros(const typename connection_types::PointCloudMessage::Ptr& apex_msg, sensor_msgs::PointCloud2::Ptr &out) {
        boost::apply_visitor (Export(out), apex_msg->value);
    }
};



RegisterPointCloudPlugin::RegisterPointCloudPlugin()
{
}

void RegisterPointCloudPlugin::init(CsApexCore& core)
{
    Tag::createIfNotExists("PointCloud");

    ConnectionTypeManager::registerMessage<connection_types::PointCloudMessage>();

    RosMessageConversion::registerConversion<sensor_msgs::PointCloud2, connection_types::PointCloudMessage, Sensor2Cloud >();
}
