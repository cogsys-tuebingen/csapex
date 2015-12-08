#ifndef PARAMETER_BUILDER_H
#define PARAMETER_BUILDER_H

/// COMPONENT
#include <csapex/param/parameter.h>

namespace csapex
{
namespace param
{

class ParameterBuilder
{
public:
    ParameterBuilder(const std::shared_ptr<Parameter>& param);

    ParameterBuilder& description(const std::string& description);
    ParameterBuilder& name(const std::string& name);

    std::shared_ptr<Parameter> build();
    operator std::shared_ptr<Parameter>();

private:
    std::shared_ptr<Parameter> param_;
};


} // param
} // csapex

#endif // PARAMETER_BUILDER_H

