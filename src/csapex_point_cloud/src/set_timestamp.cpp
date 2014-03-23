/// HEADER
#include "set_timestamp.h"

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex_transform/time_stamp_message.h>

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

void SetTimeStamp::setup()
{
    setSynchronizedInputs(true);

    input_ = addInput<connection_types::PointCloudMessage>("PointCloud");
    input_frame_ = addInput<connection_types::DirectMessage<std::string> >("Frame", true, true);
    input_time_ = addInput<connection_types::TimeStampMessage>("Time");

    output_ = addOutput<connection_types::PointCloudMessage>("PointCloud");
}

void SetTimeStamp::process()
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

    if(input_frame_->isConnected() && input_frame_->hasMessage()) {
        cloud->header.frame_id = input_frame_->getMessage<connection_types::DirectMessage<std::string> >()->value;
    }

    msg->value = cloud;
    output_->publish(msg);
}
