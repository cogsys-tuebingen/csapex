#ifndef PARAMETER_DESCRIPTION_H
#define PARAMETER_DESCRIPTION_H

/// SYSTEM
#include <string>

namespace csapex {
namespace param
{
class ParameterDescription
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
