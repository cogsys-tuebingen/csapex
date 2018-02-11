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
protected:
    explicit ParameterImplementation(const std::string& name, const ParameterDescription& description)
        : Parameter(name, description)
    {

    }

    ParameterImplementation(const Parameter& other)
        : Parameter(other)
    {

    }

    std::shared_ptr<Clonable> makeEmptyClone() const override
    {
        return std::make_shared<I>();
    }

public:
    static std::string typeName()
    {
        return type2nameWithoutNamespace(typeid(I));
    }

    static const int NUMERICAL_ID = _NUMERICAL_ID;

    virtual int ID() const override { return NUMERICAL_ID; }
};

}

}

#endif // PARAMETER_IMPL_H
