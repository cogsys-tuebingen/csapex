/// HEADER
#include <csapex/param/trigger_parameter.h>

/// SYSTEM
#include <yaml-cpp/yaml.h>

using namespace csapex;
using namespace param;

TriggerParameter::TriggerParameter()
    : Parameter("noname", ParameterDescription())
{
}

TriggerParameter::TriggerParameter(const std::string &name, const ParameterDescription& description)
    : Parameter(name, description)
{
}

TriggerParameter::~TriggerParameter()
{

}


const std::type_info& TriggerParameter::type() const
{
    return typeid(void);
}

std::string TriggerParameter::toStringImpl() const
{
    std::stringstream v;

    return std::string("[trigger]");
}

void TriggerParameter::get_unsafe(boost::any& out) const
{
    throw std::runtime_error("cannot read TriggerParameter");
}


bool TriggerParameter::set_unsafe(const boost::any &/*v*/)
{
    return false;
}


void TriggerParameter::doSetValueFrom(const Parameter &/*other*/)
{
}

void TriggerParameter::doClone(const Parameter &/*other*/)
{
}

void TriggerParameter::doSerialize(YAML::Node&) const
{
}

void TriggerParameter::doDeserialize(const YAML::Node& n)
{
}

void TriggerParameter::trigger()
{
    triggerChange();
}

bool TriggerParameter::hasState() const
{
    return false;
}
