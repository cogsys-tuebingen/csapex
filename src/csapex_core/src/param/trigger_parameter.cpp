/// HEADER
#include <csapex/param/trigger_parameter.h>

/// PROJECT
#include <csapex/param/register_parameter.h>
#include <csapex/serialization/io/std_io.h>

/// SYSTEM
#include <yaml-cpp/yaml.h>

CSAPEX_REGISTER_PARAM(TriggerParameter)

using namespace csapex;
using namespace param;

TriggerParameter::TriggerParameter() : ParameterImplementation("noname", ParameterDescription())
{
}

TriggerParameter::TriggerParameter(const std::string& name, const ParameterDescription& description) : ParameterImplementation(name, description)
{
}

TriggerParameter::~TriggerParameter()
{
}

TriggerParameter& TriggerParameter::operator=(const TriggerParameter& p)
{
    Parameter::operator =(static_cast<const Parameter&>(p));

    // do nothing
    return *this;
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

bool TriggerParameter::set_unsafe(const boost::any& /*v*/)
{
    trigger();
    return true;
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
