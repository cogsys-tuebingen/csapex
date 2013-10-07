/// HEADER
#include "transform_cloud.h"

/// PROJECT
#include <csapex/model/box.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex_transform/transform_message.h>

/// SYSTEM
#include <pluginlib/class_list_macros.h>
#include <boost/mpl/for_each.hpp>
#include <pcl_ros/transforms.h>

PLUGINLIB_EXPORT_CLASS(csapex::TransformCloud, csapex::BoxedObject)

using namespace csapex;
using namespace csapex::connection_types;

TransformCloud::TransformCloud()
{
    addTag(Tag::get("PointCloud"));
}

void TransformCloud::fill(QBoxLayout *layout)
{
    box_->setSynchronizedInputs(true);

    input_cloud_ = new ConnectorIn(box_, 0);
    input_cloud_->setLabel("PointCloud");
    input_cloud_->setType(PointCloudMessage::make());

    input_transform_ = new ConnectorIn(box_, 1);
    input_transform_->setLabel("Transformation");
    input_transform_->setType(TransformMessage::make());

    output_ = new ConnectorOut(box_, 0);
    output_->setLabel("PointCloud");
    output_->setType(PointCloudMessage::make());

    box_->addInput(input_cloud_);
    box_->addInput(input_transform_);
    box_->addOutput(output_);
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
