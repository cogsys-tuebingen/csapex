#include "transform_from_models.h"

/// COMPONENT
#include <csapex_transform/transform_message.h>
#include <csapex_core_plugins/ros_message_conversion.h>

/// PROJECT
#include <csapex/model/connector_out.h>
#include <csapex/model/connector_in.h>
#include <csapex_core_plugins/vector_message.h>
#include <csapex_point_cloud/model_message.h>
#include <utils_param/parameter_factory.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::TransformFromModels, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

TransformFromModels::TransformFromModels()
{
    addTag(Tag::get("Transform"));
}


void TransformFromModels::process()
{
//    // Copied from static_transform.cpp remove agian
//    double roll = param<double>("roll");
//    double pitch = param<double>("pitch");
//    double yaw = param<double>("yaw");
//    double x = param<double>("dx");
//    double y = param<double>("dy");
//    double z = param<double>("dz");

//    connection_types::TransformMessage::Ptr msg(new connection_types::TransformMessage);
//    msg->value = tf::Transform(tf::createQuaternionFromRPY(roll, pitch, yaw), tf::Vector3(x, y, z));
//    output_->publish(msg);
}

void TransformFromModels::setup()
{
    setSynchronizedInputs(true);
    input_models_ref_ = addInput<GenericVectorMessage, ModelMessage >("Reference Models");
    input_models_new_ = addInput<GenericVectorMessage, ModelMessage >("New Models");
    output_ = addOutput<connection_types::TransformMessage>("Transformation");
    //output_text_ = addOutput<StringMessage>("String");

    addParameter(param::ParameterFactory::declareBool("publish marker", true));
}
