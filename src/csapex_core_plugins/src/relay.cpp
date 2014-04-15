/// HEADER
#include "relay.h"

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex/model/connection_type.h>
#include <csapex/model/message.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::Relay, csapex::Node)

using namespace csapex;

Relay::Relay()
    : input_(NULL), output_(NULL)
{
    addTag(Tag::get("Buffer"));
    addTag(Tag::get("General"));
}

QIcon Relay::getIcon() const
{
    return QIcon(":/buffer.png");
}

void Relay::setup()
{
    setSynchronizedInputs(true);

    input_ = addInput<connection_types::AnyMessage>("Anything");
    output_ = addOutput<connection_types::AnyMessage>("Same as input");
}

void Relay::process()
{
    ConnectionType::Ptr msg = input_->getMessage<ConnectionType>();

    output_->setType(input_->getType());
    output_->publish(msg);
}

