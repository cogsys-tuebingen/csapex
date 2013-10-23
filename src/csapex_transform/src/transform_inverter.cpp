/// HEADER
#include "transform_inverter.h"

/// COMPONENT
#include <csapex_transform/transform_message.h>

/// PROJECT
#include <csapex/model/box.h>
#include <csapex/model/connector_out.h>
#include <csapex/model/connector_in.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::TransformInverter, csapex::BoxedObject)

using namespace csapex;

TransformInverter::TransformInverter()
{
    addTag(Tag::get("Transform"));
}
void TransformInverter::allConnectorsArrived()
{
    connection_types::TransformMessage::Ptr trafo = input_->getMessage<connection_types::TransformMessage>();
    connection_types::TransformMessage::Ptr msg(new connection_types::TransformMessage);
    msg->value = trafo->value.inverse();
    output_->publish(msg);
}


void TransformInverter::fill(QBoxLayout* layout)
{
    box_->setSynchronizedInputs(true);

    input_ = box_->addInput<connection_types::TransformMessage>("T");

    output_ = box_->addOutput<connection_types::TransformMessage>("T^-1");
}
