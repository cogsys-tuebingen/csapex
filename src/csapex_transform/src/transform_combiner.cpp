/// HEADER
#include "transform_combiner.h"

/// COMPONENT
#include <csapex_transform/transform_message.h>

/// PROJECT

#include <csapex/model/connector_out.h>
#include <csapex/model/connector_in.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::TransformCombiner, csapex::BoxedObject)

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
    setSynchronizedInputs(true);

    input_a_ = addInput<connection_types::TransformMessage>("A");
    input_b_ = addInput<connection_types::TransformMessage>("B");

    output_ = addOutput<connection_types::TransformMessage>("A*B");
}
