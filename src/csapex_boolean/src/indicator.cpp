/// HEADER
#include "indicator.h"

/// COMPONENT
#include <csapex_boolean/boolean_message.h>

/// PROJECT
#include <csapex/box.h>
#include <csapex/connector_out.h>
#include <csapex/connector_in.h>

/// SYSTEM
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(csapex::boolean::Indicator, csapex::BoxedObject)

using namespace csapex;
using namespace csapex::boolean;
using namespace csapex::connection_types;

Indicator::Indicator()
{
    addTag(Tag::get("Boolean"));
}

void Indicator::fill(QBoxLayout *layout)
{
    in = new ConnectorIn(box_, 0);
    in->setType(csapex::connection_types::BooleanMessage::make());
    in->setLabel("Signal");
    box_->addInput(in);

    indicator_ = new QCheckBox("signal");

    std::string style = "QCheckBox {spacing: 5px;} QCheckBox::indicator { width: 16px; height: 16px; }";
    style += "QCheckBox::indicator:unchecked { image: none; background-color: #FF0000; }";
    style += "QCheckBox::indicator:checked { image: none; background-color: #00FF00; }";

    indicator_->setStyleSheet(style.c_str());
    layout->addWidget(indicator_);
}

void Indicator::messageArrived(ConnectorIn *source)
{
    ConnectionType::Ptr msg = in->getMessage();
    BooleanMessage::Ptr a = boost::dynamic_pointer_cast<BooleanMessage> (msg);
    assert(a);

    indicator_->setChecked(a->value);

    indicator_->setText(a->value ? "true" : "false");
}
