#ifndef PARAMETER_IMPL_H
#define PARAMETER_IMPL_H

/// PROJECT
#include <csapex/param/parameter.h>
#include <csapex/utility/type.h>

namespace csapex
{
namespace param
{
template <typename I, int _NUMERICAL_ID>
class ParameterImplementation : public Parameter
{
    CLONABLE_IMPLEMENTATION_NO_ASSIGNMENT(I);

protected:
    explicit ParameterImplementation(const std::string& name, const ParameterDescription& description) : Parameter(name, description)
    {
    }

    ParameterImplementation(const Parameter& other) : Parameter(other)
    {
    }

    virtual void cloneData(const I& other)
    {
        dynamic_cast<I&>(*this) = other;
    }

public:
    static std::string typeName()
    {
        return type2nameWithoutNamespace(typeid(I));
    }

    static const int NUMERICAL_ID = _NUMERICAL_ID;

    virtual int ID() const override
    {
        return NUMERICAL_ID;
    }
};

}  // namespace param

}  // namespace csapex

#endif  // PARAMETER_IMPL_H
