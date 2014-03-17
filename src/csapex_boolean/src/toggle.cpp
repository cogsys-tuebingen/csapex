/// HEADER
#include "toggle.h"

/// COMPONENT
#include <csapex_boolean/boolean_message.h>

/// PROJECT
#include <csapex/model/connector_out.h>
#include <csapex/model/connector_in.h>
#include <utils_param/parameter_factory.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::boolean::Toggle, csapex::Node)

using namespace csapex;
using namespace csapex::boolean;

Toggle::Toggle()
{
    addTag(Tag::get("Boolean"));
    addParameter(param::ParameterFactory::declareBool("true", true),
                 boost::bind(&Toggle::setSignal, this));
}

void Toggle::process()
{

}

void Toggle::setup()
{
    out = addOutput<connection_types::BooleanMessage>("Signal");
}

void Toggle::tick()
{
    csapex::connection_types::BooleanMessage::Ptr msg(new csapex::connection_types::BooleanMessage);
    msg->value = signal_;
    out->publish(msg);
}

void Toggle::setSignal()
{
    signal_ = param<bool>("true");
}
