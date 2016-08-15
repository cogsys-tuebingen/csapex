#ifndef PARAMETER_BUILDER_H
#define PARAMETER_BUILDER_H

/// COMPONENT
#include <csapex/param/parameter.h>
#include <csapex/csapex_param_export.h>

namespace csapex
{
namespace param
{

class CSAPEX_PARAM_EXPORT ParameterBuilder
{
public:
    ParameterBuilder(const std::shared_ptr<Parameter>& param);

    ParameterBuilder& description(const std::string& description);
    ParameterBuilder& name(const std::string& name);

    template <class T>
    std::shared_ptr<T> build() {
        return std::dynamic_pointer_cast<T>(build());
    }

    std::shared_ptr<Parameter> build();
    operator std::shared_ptr<Parameter>();

private:
    std::shared_ptr<Parameter> param_;
};


} // param
} // csapex

#endif // PARAMETER_BUILDER_H

