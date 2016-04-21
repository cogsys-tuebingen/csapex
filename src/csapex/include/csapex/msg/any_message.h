#ifndef ANY_MESSAGE_H
#define ANY_MESSAGE_H

/// COMPONENT
#include <csapex/msg/message.h>

namespace csapex
{
namespace connection_types
{

struct AnyMessage : public Message
{
public:
    typedef std::shared_ptr<AnyMessage> Ptr;

public:
    AnyMessage();

public:
    virtual Token::Ptr clone() const override;
    virtual Token::Ptr toType() const override;

    bool canConnectTo(const Token* other_side) const override;
    bool acceptsConnectionFrom(const Token* other_side) const override;
};
template <>
struct type<AnyMessage> {
    static std::string name() {
        return "Anything";
    }
};
}
}

/// YAML
namespace YAML {
template<>
struct convert<csapex::connection_types::AnyMessage> {
  static Node encode(const csapex::connection_types::AnyMessage& rhs);
  static bool decode(const Node& node, csapex::connection_types::AnyMessage& rhs);
};


}

#endif // ANY_MESSAGE_H

