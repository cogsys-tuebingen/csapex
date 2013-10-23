/// HEADER
#include "relay.h"

/// PROJECT
#include <csapex/model/box.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex/model/connection_type.h>
#include <csapex/model/message.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::Relay, csapex::BoxedObject)

using namespace csapex;

Relay::Relay()
    : input_(NULL), output_(NULL)
{
    addTag(Tag::get("Buffer"));
    addTag(Tag::get("General"));
    setIcon(QIcon(":/buffer.png"));
}

void Relay::fill(QBoxLayout *layout)
{
    if(input_ == NULL) {
        input_ = box_->addInput<connection_types::AnyMessage>("Anything");
        output_ = box_->addOutput<connection_types::AnyMessage>("Same as input");
    }
}

void Relay::messageArrived(ConnectorIn *source)
{
    ConnectionType::Ptr msg = source->getMessage<ConnectionType>();

    output_->setType(input_->getType());

    output_->publish(msg);
}

