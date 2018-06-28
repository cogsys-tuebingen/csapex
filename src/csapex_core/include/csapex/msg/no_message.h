#ifndef NO_MESSAGE_H
#define NO_MESSAGE_H

/// COMPONENT
#include <csapex/msg/marker_message.h>

namespace csapex
{
namespace connection_types
{

struct CSAPEX_CORE_EXPORT NoMessage : public MarkerMessage
{
protected:
    CLONABLE_IMPLEMENTATION(NoMessage);

public:
    typedef std::shared_ptr<NoMessage> Ptr;

public:
    NoMessage();

public:
    void serialize(SerializationBuffer &data, SemanticVersion& version) const override;
    void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;
};

template <>
struct type<NoMessage> {
    static std::string name() {
        return "Nothing";
    }
};
}


template <>
inline std::shared_ptr<connection_types::NoMessage> makeEmpty<connection_types::NoMessage>()
{
    static std::shared_ptr<connection_types::NoMessage> instance(new connection_types::NoMessage);
    return instance;
}

}

/// YAML
namespace YAML {
template<>
struct CSAPEX_CORE_EXPORT convert<csapex::connection_types::NoMessage> {
  static Node encode(const csapex::connection_types::NoMessage& rhs);
  static bool decode(const Node& node, csapex::connection_types::NoMessage& rhs);
};


}

#endif // NO_MESSAGE_H

