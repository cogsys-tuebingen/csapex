/// HEADER
#include "relay.h"

/// PROJECT
#include <csapex/box.h>
#include <csapex/connector_in.h>
#include <csapex/connector_out.h>
#include <csapex/connection_type.h>
#include <csapex/message.h>

/// SYSTEM
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(csapex::Relay, csapex::BoxedObject)

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
        input_ = new ConnectorIn(box_, 0);
        input_->setLabel("Anything");
        input_->setType(connection_types::AnyMessage::make());

        output_ = new ConnectorOut(box_, 0);
        output_->setLabel("Same as input");
        output_->setType(connection_types::AnyMessage::make());

        box_->addInput(input_);
        box_->addOutput(output_);
    }
}

void Relay::messageArrived(ConnectorIn *source)
{
    ConnectionType::Ptr msg = source->getMessage();

    output_->setType(input_->getType());

    output_->publish(msg);
}

