#ifndef RESPONSE_IMPL_HPP
#define RESPONSE_IMPL_HPP

/// PROJECT
#include <csapex/io/response.h>
#include <csapex/utility/type.h>

namespace csapex
{

template <typename I>
class ResponseImplementation : public Response
{
protected:
    ResponseImplementation()
    {
    }

    std::shared_ptr<Clonable> makeEmptyClone() const override
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

#endif // RESPONSE_IMPL_HPP
