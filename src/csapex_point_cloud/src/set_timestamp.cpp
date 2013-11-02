/// HEADER
#include "set_timestamp.h"

/// PROJECT

#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex_transform/time_stamp_message.h>
#include <csapex_core_plugins/string_message.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::SetTimeStamp, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

SetTimeStamp::SetTimeStamp()
{
    addTag(Tag::get("PointCloud"));
    addTag(Tag::get("Time"));
}

void SetTimeStamp::fill(QBoxLayout *layout)
{
    setSynchronizedInputs(true);

    input_ = addInput<connection_types::PointCloudMessage>("PointCloud");
    input_frame_ = addInput<connection_types::StringMessage>("Frame", true);
    input_time_ = addInput<connection_types::TimeStampMessage>("Time");

    output_ = addOutput<connection_types::PointCloudMessage>("PointCloud");
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
