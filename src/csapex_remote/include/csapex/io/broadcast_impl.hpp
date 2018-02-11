#ifndef BROADCAST_IMPL_HPP
#define BROADCAST_IMPL_HPP

/// PROJECT
#include <csapex/io/broadcast_message.h>
#include <csapex/utility/type.h>

namespace csapex
{

template <typename I>
class BroadcastImplementation : public BroadcastMessage
{
protected:
    BroadcastImplementation()
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

#endif // BROADCAST_IMPL_HPP
