/// HEADER
#include <csapex/param/parameter_builder.h>

/// COMPONENT
#include <csapex/param/parameter.h>

using namespace csapex;
using namespace param;

ParameterBuilder::ParameterBuilder(const std::shared_ptr<Parameter>& param)
    : param_(param)
{
}

ParameterBuilder& ParameterBuilder::name(const std::string& name)
{
    Parameter& p = *param_;
    p.setName(name);

    return *this;
}


ParameterBuilder& ParameterBuilder::description(const std::string& description)
{
    Parameter& p = *param_;
    p.setDescription(param::ParameterDescription(description));

    return *this;
}

std::shared_ptr<Parameter> ParameterBuilder::build()
{
    return param_;
}

ParameterBuilder::operator std::shared_ptr<Parameter>()
{
    return build();
}
