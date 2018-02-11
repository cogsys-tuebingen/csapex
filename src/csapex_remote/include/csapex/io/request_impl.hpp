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
    RequestImplementation(uint8_t id)
        : Request(id)
    {
    }

    std::shared_ptr<Clonable> makeEmptyClone() const
    {
        return std::make_shared<I>(0);
    }
};

}

#endif // REQUEST_IMPL_HPP
