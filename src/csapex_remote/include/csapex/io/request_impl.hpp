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
    CLONABLE_IMPLEMENTATION_CONSTRUCTOR(I, 0);

protected:
    RequestImplementation(uint8_t id) : Request(id)
    {
    }
};

}  // namespace csapex

#endif  // REQUEST_IMPL_HPP
