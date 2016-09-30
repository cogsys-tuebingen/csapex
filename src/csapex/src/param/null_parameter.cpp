/// HEADER
#include <csapex/param/null_parameter.h>

/// SYSTEM
#include <yaml-cpp/yaml.h>

using namespace csapex;
using namespace param;

NullParameter::NullParameter()
    : Parameter("null", ParameterDescription())
{
}


NullParameter::NullParameter(const std::string &name, const ParameterDescription& description)
    : Parameter(name, description)
{
}

NullParameter::~NullParameter()
{

}

const std::type_info& NullParameter::type() const
{
    return typeid(void);
}

std::string NullParameter::toStringImpl() const
{
    return std::string("[null]");
}

void NullParameter::get_unsafe(boost::any& out) const
{
    throw std::runtime_error("cannot use null parameters");
}


bool NullParameter::set_unsafe(const boost::any& /*v*/)
{
    throw std::runtime_error("cannot use null parameters");
}


void NullParameter::doSetValueFrom(const Parameter &/*other*/)
{
    throw std::runtime_error("cannot use null parameters");
}

void NullParameter::doClone(const Parameter &/*other*/)
{
    throw std::runtime_error("cannot use null parameters");
}

void NullParameter::doSerialize(YAML::Node& /*n*/) const
{
}

void NullParameter::doDeserialize(const YAML::Node& /*n*/)
{
}

