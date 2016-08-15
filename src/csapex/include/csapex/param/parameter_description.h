#ifndef PARAMETER_DESCRIPTION_H
#define PARAMETER_DESCRIPTION_H

/// COMPONENT
#include <csapex/csapex_param_export.h>

/// SYSTEM
#include <string>

namespace csapex {
namespace param
{
class CSAPEX_PARAM_EXPORT ParameterDescription
{
public:
    explicit ParameterDescription(const std::string& toString);
    explicit ParameterDescription();

    std::string toString() const;
    bool empty() const;

private:
    std::string description_;
};
}
}

#endif // PARAMETER_DESCRIPTION_H
