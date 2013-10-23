/// HEADER
#include "indicator.h"

/// COMPONENT
#include <csapex_boolean/boolean_message.h>

/// PROJECT
#include <csapex/model/box.h>
#include <csapex/model/connector_out.h>
#include <csapex/model/connector_in.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::boolean::Indicator, csapex::BoxedObject)

using namespace csapex;
using namespace csapex::boolean;
using namespace csapex::connection_types;

Indicator::Indicator()
{
    addTag(Tag::get("Boolean"));
}

void Indicator::fill(QBoxLayout *layout)
{
    setSynchronizedInputs(true);

    in = addInput<BooleanMessage>("Signal");

    indicator_ = new QCheckBox("signal");

    std::string style = "QCheckBox {spacing: 5px;} QCheckBox::indicator { width: 16px; height: 16px; }";
    style += "QCheckBox::indicator:unchecked { image: none; background-color: #FF0000; }";
    style += "QCheckBox::indicator:checked { image: none; background-color: #00FF00; }";

    indicator_->setStyleSheet(style.c_str());
    layout->addWidget(indicator_);
}

void Indicator::allConnectorsArrived()
{
    BooleanMessage::Ptr a = in->getMessage<BooleanMessage>();
    assert(a);

    indicator_->setChecked(a->value);

    indicator_->setText(a->value ? "true" : "false");
}
