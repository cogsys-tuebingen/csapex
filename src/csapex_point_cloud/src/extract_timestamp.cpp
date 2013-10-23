/// HEADER
#include "extract_timestamp.h"

/// PROJECT
#include <csapex/model/box.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex_transform/time_stamp_message.h>
#include <csapex_core_plugins/string_message.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::ExtractTimeStamp, csapex::BoxedObject)

using namespace csapex;
using namespace csapex::connection_types;

ExtractTimeStamp::ExtractTimeStamp()
{
    addTag(Tag::get("PointCloud"));
    addTag(Tag::get("Time"));
}

void ExtractTimeStamp::fill(QBoxLayout *layout)
{
    setSynchronizedInputs(true);

    input_ = addInput<PointCloudMessage>("PointCloud");

    output_ = addOutput<TimeStampMessage>("Time");
    output_frame_ = addOutput<StringMessage>("Target Frame");
}

void ExtractTimeStamp::allConnectorsArrived()
{
    PointCloudMessage::Ptr msg(input_->getMessage<PointCloudMessage>());

    boost::apply_visitor (PointCloudMessage::Dispatch<ExtractTimeStamp>(this), msg->value);
}

template <class PointT>
void ExtractTimeStamp::inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    connection_types::TimeStampMessage::Ptr time(new connection_types::TimeStampMessage);
    time->value = time->value.fromNSec(cloud->header.stamp * 1000);
    output_->publish(time);

    connection_types::StringMessage::Ptr frame(new connection_types::StringMessage);
    frame->value = cloud->header.frame_id;
    output_frame_->publish(frame);
}
