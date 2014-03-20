/// HEADER
#include "double_input.h"

/// PROJECT
#include <csapex/model/connector_out.h>
#include <csapex_core_plugins/double_message.h>
#include <utils_param/parameter_factory.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>
#include <QPushButton>
#include <QBoxLayout>

CSAPEX_REGISTER_CLASS(csapex::DoubleInput, csapex::Node)

using namespace csapex;

DoubleInput::DoubleInput()
{
    addTag(Tag::get("Input"));
    addTag(Tag::get("General"));
    setIcon(QIcon(":/pencil.png"));

    addParameter(param::ParameterFactory::declareValue<double>("value", 0.0));
    addParameter(param::ParameterFactory::declareTrigger("publish"), boost::bind(&DoubleInput::process, this));
    addParameter(param::ParameterFactory::declareBool("tick", false));
}

void DoubleInput::tick()
{
    if(param<bool>("tick")) {
        process();
    }
}

void DoubleInput::setup()
{
    setSynchronizedInputs(true);

    out_ = addOutput<connection_types::DoubleMessage>("Double");
}

void DoubleInput::process()
{
    connection_types::DoubleMessage::Ptr msg(new connection_types::DoubleMessage);
    msg->value = param<double>("value");

    out_->publish(msg);
}
