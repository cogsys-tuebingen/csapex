/// HEADER
#include "transform_inverter.h"

/// COMPONENT
#include <csapex_transform/transform_message.h>

/// PROJECT
#include <csapex/model/box.h>
#include <csapex/model/connector_out.h>
#include <csapex/model/connector_in.h>

/// SYSTEM
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(csapex::TransformInverter, csapex::BoxedObject)

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

    output_ = new ConnectorOut(box_, 0);
    output_->setType(connection_types::TransformMessage::make());
    box_->addOutput(output_);

    input_ = new ConnectorIn(box_, 0);
    input_->setType(connection_types::TransformMessage::make());
    box_->addInput(input_);
}
