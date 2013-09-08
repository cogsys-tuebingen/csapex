/// HEADER
#include "toggle.h"

/// COMPONENT
#include <csapex_boolean/boolean_message.h>

/// PROJECT
#include <csapex/box.h>
#include <csapex/connector_out.h>
#include <csapex/connector_in.h>

/// SYSTEM
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(csapex::boolean::Toggle, csapex::BoxedObject)

using namespace csapex;
using namespace csapex::boolean;

Toggle::Toggle()
{
    addTag(Tag::get("Boolean"));
}

void Toggle::fill(QBoxLayout *layout)
{
    out = new ConnectorOut(box_, 0);
    out->setType(csapex::connection_types::BooleanMessage::make());
    out->setLabel("Signal");
    box_->addOutput(out);

    btn = new QPushButton();
    btn->setCheckable(true);
    setSignal(false);

    layout->addWidget(btn);

    connect(btn, SIGNAL(toggled(bool)), this, SLOT(setSignal(bool)));
}

void Toggle::tick()
{
    csapex::connection_types::BooleanMessage::Ptr msg(new csapex::connection_types::BooleanMessage);
    msg->value = signal_;
    out->publish(msg);
}

void Toggle::messageArrived(ConnectorIn *source)
{
    // NO INPUT
}

void Toggle::setSignal(bool signal)
{
    signal_ = signal;

    btn->setChecked(signal);
    btn->setText(signal ? "true" : "false");
}
