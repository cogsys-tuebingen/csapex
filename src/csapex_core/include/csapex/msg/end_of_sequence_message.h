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
    void serialize(SerializationBuffer& data, SemanticVersion& version) const override;
    void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;

protected:
    EndOfSequenceMessage(const std::string& name);
};

template <>
struct type<EndOfSequenceMessage>
{
    static std::string name()
    {
        return "EndOfSequence";
    }
};

}  // namespace connection_types
}  // namespace csapex

/// YAML
namespace YAML
{
template <>
struct CSAPEX_CORE_EXPORT convert<csapex::connection_types::EndOfSequenceMessage>
{
    static Node encode(const csapex::connection_types::EndOfSequenceMessage& rhs);
    static bool decode(const Node& node, csapex::connection_types::EndOfSequenceMessage& rhs);
};
}  // namespace YAML

#endif  // END_OF_SEQUENCE_MESSAGE_H
