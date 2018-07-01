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
    CLONABLE_IMPLEMENTATION(I);

protected:
    BroadcastImplementation()
    {
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

}  // namespace csapex

#endif  // BROADCAST_IMPL_HPP
