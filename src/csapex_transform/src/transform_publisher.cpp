/// HEADER
#include "transform_publisher.h"

/// COMPONENT
#include <csapex_transform/transform_message.h>
#include <csapex_transform/time_stamp_message.h>

/// PROJECT
#include <csapex/model/connector_out.h>
#include <csapex/model/connector_in.h>
#include <utils_param/parameter_factory.h>
#include <csapex_core_plugins/ros_handler.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::TransformPublisher, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

TransformPublisher::TransformPublisher()
    : tfb_(NULL)
{
    addTag(Tag::get("Transform"));

    addParameter(param::ParameterFactory::declareText("from", "/"));
    addParameter(param::ParameterFactory::declareText("to", "/"));
}

TransformPublisher::~TransformPublisher()
{
    if(tfb_) {
        delete tfb_;
    }
}

void TransformPublisher::allConnectorsArrived()
{
    if(!ROSHandler::instance().isConnected()) {
        return;
    }

    if(!tfb_) {
        tfb_ = new tf::TransformBroadcaster;
    }

    ros::Time time;
    if(input_time->isConnected()) {
        TimeStampMessage::Ptr time_msg = input_time->getMessage<TimeStampMessage>();
        time = time_msg->value;
    } else {
        time = ros::Time::now();
    }

    TransformMessage::Ptr trafo_msg = input_transform->getMessage<TransformMessage>();

    tfb_->sendTransform(tf::StampedTransform(trafo_msg->value, time, param<std::string>("from"), param<std::string>("to")));
}


void TransformPublisher::setup()
{
    setSynchronizedInputs(true);

    input_transform = addInput<connection_types::TransformMessage>("T");
    input_time = addInput<connection_types::TimeStampMessage>("time", true);
}
