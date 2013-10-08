/// HEADER
#include "transform_combiner.h"

/// COMPONENT
#include <csapex_transform/transform_message.h>

/// PROJECT
#include <csapex/model/box.h>
#include <csapex/model/connector_out.h>
#include <csapex/model/connector_in.h>

/// SYSTEM
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(csapex::TransformCombiner, csapex::BoxedObject)

using namespace csapex;

TransformCombiner::TransformCombiner()
{
    addTag(Tag::get("Transform"));
}
void TransformCombiner::allConnectorsArrived()
{
    connection_types::TransformMessage::Ptr a = input_a_->getMessage<connection_types::TransformMessage>();
    connection_types::TransformMessage::Ptr b = input_b_->getMessage<connection_types::TransformMessage>();

    connection_types::TransformMessage::Ptr msg(new connection_types::TransformMessage);
    msg->value = a->value * b->value;
    output_->publish(msg);
}


void TransformCombiner::fill(QBoxLayout* layout)
{
    box_->setSynchronizedInputs(true);

    output_ = new ConnectorOut(box_, 0);
    output_->setType(connection_types::TransformMessage::make());
    box_->addOutput(output_);

    input_a_ = new ConnectorIn(box_, 0);
    input_a_->setType(connection_types::TransformMessage::make());
    box_->addInput(input_a_);

    input_b_ = new ConnectorIn(box_, 1);
    input_b_->setType(connection_types::TransformMessage::make());
    box_->addInput(input_b_);
}
