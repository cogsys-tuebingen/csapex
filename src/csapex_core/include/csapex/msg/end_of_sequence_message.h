#ifndef END_OF_SEQUENCE_MESSAGE_H
#define END_OF_SEQUENCE_MESSAGE_H

/// COMPONENT
#include <csapex/msg/marker_message.h>

namespace csapex
{
namespace connection_types
{

struct CSAPEX_CORE_EXPORT EndOfSequenceMessage : public MarkerMessage
{
protected:
    CLONABLE_IMPLEMENTATION(EndOfSequenceMessage);

public:
    typedef std::shared_ptr<EndOfSequenceMessage> Ptr;

public:
    EndOfSequenceMessage();

protected:
    EndOfSequenceMessage(const std::string& name);

public:
    void serialize(SerializationBuffer &data) const override;
    void deserialize(const SerializationBuffer& data) override;
};

template <>
struct type<EndOfSequenceMessage> {
    static std::string name() {
        return "EndOfSequence";
    }
};

}
}

/// YAML
namespace YAML {
template<>
struct CSAPEX_CORE_EXPORT convert<csapex::connection_types::EndOfSequenceMessage> {
  static Node encode(const csapex::connection_types::EndOfSequenceMessage& rhs);
  static bool decode(const Node& node, csapex::connection_types::EndOfSequenceMessage& rhs);
};
}

#endif // END_OF_SEQUENCE_MESSAGE_H
