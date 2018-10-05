#ifndef PARAMETER_IMPL_H
#define PARAMETER_IMPL_H

/// PROJECT
#include <csapex/param/parameter.h>
#include <csapex/utility/type.h>

namespace csapex
{
namespace param
{
template <typename I>
class ParameterImplementation : public Parameter
{
    CLONABLE_IMPLEMENTATION_NO_ASSIGNMENT(I);

public:
    static std::string typeName()
    {
        return type2nameWithoutNamespace(typeid(I));
    }

    virtual std::string getParameterType() const override
    {
        return serializationName<I>();
    }

protected:
    explicit ParameterImplementation(const std::string& name, const ParameterDescription& description) : Parameter(name, description)
    {
    }

    ParameterImplementation(const Parameter& other) : Parameter(other)
    {
    }

    virtual bool cloneData(const I& other)
    {
        dynamic_cast<I&>(*this) = other;
        return true;
    }
};

}  // namespace param

}  // namespace csapex

#endif  // PARAMETER_IMPL_H
