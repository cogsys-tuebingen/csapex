/// HEADER
#include <csapex/msg/end_of_program_message.h>

/// PROJECT
#include <csapex/utility/register_msg.h>

CSAPEX_REGISTER_MESSAGE(csapex::connection_types::EndOfProgramMessage)

using namespace csapex;
using namespace connection_types;

EndOfProgramMessage::EndOfProgramMessage() : MarkerMessage(type<EndOfProgramMessage>::name(), 0)
{
}

void EndOfProgramMessage::serialize(SerializationBuffer& data, SemanticVersion& version) const
{
}
void EndOfProgramMessage::deserialize(const SerializationBuffer& data, const SemanticVersion& version)
{
}


/// YAML
namespace YAML
{
Node convert<csapex::connection_types::EndOfProgramMessage>::encode(const csapex::connection_types::EndOfProgramMessage& rhs)
{
    return convert<csapex::connection_types::Message>::encode(rhs);
}

bool convert<csapex::connection_types::EndOfProgramMessage>::decode(const Node& node, csapex::connection_types::EndOfProgramMessage& rhs)
{
    if (!node.IsMap()) {
        return false;
    }
    convert<csapex::connection_types::Message>::decode(node, rhs);
    return true;
}
}  // namespace YAML
