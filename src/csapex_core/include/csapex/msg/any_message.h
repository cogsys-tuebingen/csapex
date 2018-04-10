#ifndef ANY_MESSAGE_H
#define ANY_MESSAGE_H

/// COMPONENT
#include <csapex/msg/message.h>
#include <csapex_core/csapex_core_export.h>

namespace csapex
{
namespace connection_types
{

struct CSAPEX_CORE_EXPORT AnyMessage : public Message
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


    std::shared_ptr<Clonable> makeEmptyClone() const override
    {
        return std::shared_ptr<Clonable>(new AnyMessage);
    }
    void serialize(SerializationBuffer &data) const override;
    void deserialize(const SerializationBuffer& data) override;
};

template <>
struct type<AnyMessage> {
    static std::string name() {
        return "Anything";
    }
};

template <>
inline std::shared_ptr<AnyMessage> makeEmpty<AnyMessage>()
{
    static std::shared_ptr<AnyMessage> instance(new AnyMessage);
    return instance;
}

}
}

/// YAML
namespace YAML {
template<>
struct CSAPEX_CORE_EXPORT convert<csapex::connection_types::AnyMessage> {
  static Node encode(const csapex::connection_types::AnyMessage& rhs);
  static bool decode(const Node& node, csapex::connection_types::AnyMessage& rhs);
};


}

#endif // ANY_MESSAGE_H

