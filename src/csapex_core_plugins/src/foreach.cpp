/// HEADER
#include "foreach.h"

/// COMPONENT
#include <csapex_core_plugins/vector_message.h>

/// PROJECT
#include <csapex/model/connector_out.h>
#include <csapex/model/connector_in.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::Foreach, csapex::Node)

using namespace csapex;
using namespace connection_types;

Foreach::Foreach()
{
    addTag(Tag::get("General"));
}

void Foreach::allConnectorsArrived()
{
    VectorMessage::Ptr vec = input_->getMessage<VectorMessage>();

    output_->setType(vec->getSubType());

    for(int i = 0, n = vec->value.size(); i < n; ++i) {
        output_->publish(vec->value[i]);
    }
}

void Foreach::setup()
{
    setSynchronizedInputs(true);

    input_ = addInput<VectorMessage>("Vector");

    output_ = addMultidimensionalOutput<AnyMessage>("Content");
}
