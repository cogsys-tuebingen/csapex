/// HEADER
#include "transform_cloud.h"

/// PROJECT

#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex_transform/transform_message.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>
#include <boost/mpl/for_each.hpp>
#include <pcl_ros/transforms.h>

CSAPEX_REGISTER_CLASS(csapex::TransformCloud, csapex::BoxedObject)

using namespace csapex;
using namespace csapex::connection_types;

TransformCloud::TransformCloud()
{
    addTag(Tag::get("PointCloud"));
}

void TransformCloud::fill(QBoxLayout *layout)
{
    setSynchronizedInputs(true);

    input_cloud_ = addInput<PointCloudMessage>("PointCloud");

    input_transform_ = addInput<TransformMessage>("Transformation");

    output_ = addOutput<PointCloudMessage>("PointCloud");
}

void TransformCloud::allConnectorsArrived()
{
    PointCloudMessage::Ptr msg(input_cloud_->getMessage<PointCloudMessage>());

    boost::apply_visitor (PointCloudMessage::Dispatch<TransformCloud>(this), msg->value);
}

template <class PointT>
void TransformCloud::inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    TransformMessage::Ptr transform = input_transform_->getMessage<TransformMessage>();
    const tf::Transform& t = transform->value;

    typename pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);
    pcl_ros::transformPointCloud(*cloud, *out, t);

    PointCloudMessage::Ptr msg(new PointCloudMessage);
    msg->value = out;

    output_->publish(msg);
}
