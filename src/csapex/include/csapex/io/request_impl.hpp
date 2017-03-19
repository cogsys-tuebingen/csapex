#ifndef REQUEST_IMPL_HPP
#define REQUEST_IMPL_HPP

/// PROJECT
#include <csapex/io/request.h>
#include <csapex/utility/type.h>

namespace csapex
{

template <typename I>
class RequestImplementation : public Request
{
protected:
    RequestImplementation()
    {
    }

    std::shared_ptr<Clonable> makeEmptyClone() const
    {
        return std::make_shared<I>();
    }

    std::string getType() const override
    {
        return typeName();
    }

public:
    static std::string typeName()
    {
        return type2nameWithoutNamespace(typeid(I));
    }
};

}

#endif // REQUEST_IMPL_HPP
