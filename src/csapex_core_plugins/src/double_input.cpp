/// HEADER
#include "double_input.h"

/// PROJECT
#include <csapex/model/connector_out.h>
#include <utils_param/parameter_factory.h>
#include <csapex_core_plugins/ros_message_conversion.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

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

    out_ = addOutput<double>("Double");
}

void DoubleInput::process()
{
    double val = param<double>("value");
    out_->publishIntegral(val);
}
