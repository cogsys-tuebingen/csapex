#ifndef END_OF_PROGRAM_MESSAGE_H
#define END_OF_PROGRAM_MESSAGE_H

/// COMPONENT
#include <csapex/msg/end_of_sequence_message.h>

namespace csapex
{
namespace connection_types
{
struct CSAPEX_CORE_EXPORT EndOfProgramMessage : public MarkerMessage
{
protected:
    CLONABLE_IMPLEMENTATION(EndOfProgramMessage);

public:
    typedef std::shared_ptr<EndOfProgramMessage> Ptr;

public:
    EndOfProgramMessage();
    void serialize(SerializationBuffer& data, SemanticVersion& version) const override;
    void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;
};

template <>
struct type<EndOfProgramMessage>
{
    static std::string name()
    {
        return "EndOfProgram";
    }
};

}  // namespace connection_types
}  // namespace csapex

/// YAML
namespace YAML
{
template <>
struct CSAPEX_CORE_EXPORT convert<csapex::connection_types::EndOfProgramMessage>
{
    static Node encode(const csapex::connection_types::EndOfProgramMessage& rhs);
    static bool decode(const Node& node, csapex::connection_types::EndOfProgramMessage& rhs);
};
}  // namespace YAML

#endif  // END_OF_PROGRAM_MESSAGE_H
