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
protected:
    CLONABLE_IMPLEMENTATION(AnyMessage);

public:
    typedef std::shared_ptr<AnyMessage> Ptr;

public:
    AnyMessage();

public:
    bool canConnectTo(const TokenData* other_side) const override;
    bool acceptsConnectionFrom(const TokenData* other_side) const override;

    void serialize(SerializationBuffer &data, SemanticVersion& version) const override;
    void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;
};

template <>
struct type<AnyMessage> {
    static std::string name() {
        return "Anything";
    }
};

}

template <>
inline std::shared_ptr<connection_types::AnyMessage> makeEmpty<connection_types::AnyMessage>()
{
    static std::shared_ptr<connection_types::AnyMessage> instance(new connection_types::AnyMessage);
    return instance;
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

