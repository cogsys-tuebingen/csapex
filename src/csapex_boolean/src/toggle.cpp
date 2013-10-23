/// HEADER
#include "toggle.h"

/// COMPONENT
#include <csapex_boolean/boolean_message.h>

/// PROJECT
#include <csapex/model/box.h>
#include <csapex/model/connector_out.h>
#include <csapex/model/connector_in.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::boolean::Toggle, csapex::BoxedObject)

using namespace csapex;
using namespace csapex::boolean;

Toggle::Toggle()
{
    addTag(Tag::get("Boolean"));
}

void Toggle::fill(QBoxLayout *layout)
{
    out = box_->addOutput<connection_types::BooleanMessage>("Signal");

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

void Toggle::setSignal(bool signal)
{
    signal_ = signal;

    btn->setChecked(signal);
    btn->setText(signal ? "true" : "false");
}
