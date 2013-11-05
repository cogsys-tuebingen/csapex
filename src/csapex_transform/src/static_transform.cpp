/// HEADER
#include "static_transform.h"

/// COMPONENT
#include <csapex_transform/transform_message.h>

/// PROJECT
#include <csapex/model/connector_out.h>
#include <csapex/utility/qt_helper.hpp>

/// SYSTEM
#include <tf/transform_datatypes.h>
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::StaticTransform, csapex::Node)

using namespace csapex;

StaticTransform::StaticTransform()
{
    addTag(Tag::get("Transform"));

    double p = 3.1415;
    double d = 5.0;

    addParameter(param::Parameter::declare("roll", -p, p, 0.0, 0.001));
    addParameter(param::Parameter::declare("pitch", -p, p, 0.0, 0.001));
    addParameter(param::Parameter::declare("yaw", -p, p, 0.0, 0.001));
    addParameter(param::Parameter::declare("dx", -d, d, 0.0, 0.01));
    addParameter(param::Parameter::declare("dy", -d, d, 0.0, 0.01));
    addParameter(param::Parameter::declare("dz", -d, d, 0.0, 0.01));
}

void StaticTransform::tick()
{
    double roll = param<double>("roll");
    double pitch = param<double>("pitch");
    double yaw = param<double>("yaw");
    double x = param<double>("dx");
    double y = param<double>("dy");
    double z = param<double>("dz");

    connection_types::TransformMessage::Ptr msg(new connection_types::TransformMessage);
    msg->value = tf::Transform(tf::createQuaternionFromRPY(roll, pitch, yaw), tf::Vector3(x, y, z));
    output_->publish(msg);
}


void StaticTransform::setup()
{
    setSynchronizedInputs(true);

    output_ = addOutput<connection_types::TransformMessage>("Transformation");
}
