/// HEADER
#include "set_timestamp.h"

/// PROJECT
#include <csapex/model/box.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex_transform/time_stamp_message.h>
#include <csapex_core_plugins/string_message.h>

/// SYSTEM
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(csapex::SetTimeStamp, csapex::BoxedObject)

using namespace csapex;
using namespace csapex::connection_types;

SetTimeStamp::SetTimeStamp()
{
    addTag(Tag::get("PointCloud"));
    addTag(Tag::get("Time"));
}

void SetTimeStamp::fill(QBoxLayout *layout)
{
    box_->setSynchronizedInputs(true);

    input_ = new ConnectorIn(box_, 0);
    input_->setLabel("PointCloud");
    input_->setType(connection_types::PointCloudMessage::make());

    input_time_ = new ConnectorIn(box_, 1);
    input_time_->setLabel("Time");
    input_time_->setType(connection_types::TimeStampMessage::make());

    input_frame_ = new ConnectorIn(box_, 2);
    input_frame_->setOptional(true);
    input_frame_->setLabel("Frame");
    input_frame_->setType(connection_types::StringMessage::make());

    output_ = new ConnectorOut(box_, 0);
    output_->setLabel("PointCloud");
    output_->setType(connection_types::PointCloudMessage::make());

    box_->addInput(input_);
    box_->addInput(input_time_);
    box_->addInput(input_frame_);
    box_->addOutput(output_);
}

void SetTimeStamp::allConnectorsArrived()
{
    PointCloudMessage::Ptr msg(input_->getMessage<PointCloudMessage>());

    boost::apply_visitor (PointCloudMessage::Dispatch<SetTimeStamp>(this), msg->value);
}

template <class PointT>
void SetTimeStamp::inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    connection_types::TimeStampMessage::Ptr time = input_time_->getMessage<connection_types::TimeStampMessage>();
    cloud->header.stamp = time->value.toNSec() / 1e3; // is this 1e6?

    connection_types::PointCloudMessage::Ptr msg(new connection_types::PointCloudMessage);

    if(input_frame_->isConnected()) {
        cloud->header.frame_id = input_frame_->getMessage<connection_types::StringMessage>()->value;
    }

    msg->value = cloud;
    output_->publish(msg);
}
