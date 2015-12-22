#ifndef NO_MESSAGE_H
#define NO_MESSAGE_H

/// COMPONENT
#include <csapex/msg/message.h>

namespace csapex
{
namespace connection_types
{

struct NoMessage : public Message
{
public:
    typedef std::shared_ptr<NoMessage> Ptr;

public:
    NoMessage();

public:
    virtual ConnectionType::Ptr clone() const override;
    virtual ConnectionType::Ptr toType() const override;

    bool canConnectTo(const ConnectionType* other_side) const override;
    bool acceptsConnectionFrom(const ConnectionType* other_side) const override;
};
template <>
struct type<NoMessage> {
    static std::string name() {
        return "Nothing";
    }
};

}
}

#endif // NO_MESSAGE_H

