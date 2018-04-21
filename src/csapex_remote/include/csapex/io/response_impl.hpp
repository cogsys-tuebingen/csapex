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
    CLONABLE_IMPLEMENTATION_CONSTRUCTOR(I, 0);

protected:
    ResponseImplementation(uint8_t id)
        : Response(id)
    {
    }
};

}

#endif // RESPONSE_IMPL_HPP
