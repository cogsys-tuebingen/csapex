#ifndef ANY_MESSAGE_H
#define ANY_MESSAGE_H

/// COMPONENT
#include <csapex/msg/message.h>
#include <csapex/csapex_export.h>

namespace csapex
{
namespace connection_types
{

struct CSAPEX_EXPORT AnyMessage : public Message
{
public:
    typedef std::shared_ptr<AnyMessage> Ptr;

public:
    AnyMessage();

public:
    virtual TokenData::Ptr clone() const override;
    virtual TokenData::Ptr toType() const override;

    bool canConnectTo(const TokenData* other_side) const override;
    bool acceptsConnectionFrom(const TokenData* other_side) const override;
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
struct CSAPEX_EXPORT convert<csapex::connection_types::AnyMessage> {
  static Node encode(const csapex::connection_types::AnyMessage& rhs);
  static bool decode(const Node& node, csapex::connection_types::AnyMessage& rhs);
};


}

#endif // ANY_MESSAGE_H

