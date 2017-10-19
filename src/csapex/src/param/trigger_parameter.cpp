/// HEADER
#include <csapex/param/trigger_parameter.h>

/// PROJECT
#include <csapex/serialization/parameter_serializer.h>
#include <csapex/serialization/serialization_buffer.h>

/// SYSTEM
#include <yaml-cpp/yaml.h>

CSAPEX_REGISTER_PARAMETER_SERIALIZER(TriggerParameter)

using namespace csapex;
using namespace param;

TriggerParameter::TriggerParameter()
    : ParameterImplementation("noname", ParameterDescription())
{
}

TriggerParameter::TriggerParameter(const std::string &name, const ParameterDescription& description)
    : ParameterImplementation(name, description)
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
    trigger();
    return true;
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
