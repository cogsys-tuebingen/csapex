/// HEADER
#include "time_offset.h"

/// COMPONENT
#include <csapex_transform/time_stamp_message.h>

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex/utility/qt_helper.hpp>
#include <utils_param/parameter_factory.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::TimeOffset, csapex::Node)

using namespace csapex;

TimeOffset::TimeOffset()
{
    addTag(Tag::get("Time"));

    addParameter(param::ParameterFactory::declareRange<double>("offset", -5000.0, 5000.0, 0.0, 0.5));
}


void TimeOffset::allConnectorsArrived()
{
    connection_types::TimeStampMessage::Ptr in = input_->getMessage<connection_types::TimeStampMessage>();
    connection_types::TimeStampMessage::Ptr time(new connection_types::TimeStampMessage);

    double offset = param<double>("offset");

    if(in->value.toNSec() != 0) {
        std::cerr << in->value.toNSec() << " + " << offset << " * " << 1e6 << " = " << (in->value.toNSec() + offset * 1e6) << std::endl,
                time->value = time->value.fromNSec((in->value.toNSec() + offset * 1000000));
        setError(false);
    } else {
        setError(true, "Time is 0, using current time as base", EL_WARNING);
        ros::Time now = ros::Time::now();
        time->value = now - ros::Duration(0, offset * 1000000);
    }
    output_->publish(time);
}

void TimeOffset::setup()
{
    setSynchronizedInputs(true);

    input_ = addInput<connection_types::TimeStampMessage>("Time");
    output_ = addOutput<connection_types::TimeStampMessage>("Time");
}
