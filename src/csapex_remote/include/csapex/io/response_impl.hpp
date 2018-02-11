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
    ResponseImplementation(uint8_t id)
        : Response(id)
    {
    }

    std::shared_ptr<Clonable> makeEmptyClone() const override
    {
        return std::make_shared<I>(0);
    }
};

}

#endif // RESPONSE_IMPL_HPP
