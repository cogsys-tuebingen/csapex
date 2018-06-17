/// HEADER
#include <csapex/msg/end_of_sequence_message.h>

/// PROJECT
#include <csapex/utility/register_msg.h>

CSAPEX_REGISTER_MESSAGE(csapex::connection_types::EndOfSequenceMessage)

using namespace csapex;
using namespace connection_types;

EndOfSequenceMessage::EndOfSequenceMessage()
    : MarkerMessage(type<EndOfSequenceMessage>::name(), 0)
{}

EndOfSequenceMessage::EndOfSequenceMessage(const std::string& name)
    : MarkerMessage(name, 0)
{}


void EndOfSequenceMessage::serialize(SerializationBuffer &data, SemanticVersion& version) const
{
}
void EndOfSequenceMessage::deserialize(const SerializationBuffer& data, const SemanticVersion& version)
{
}

/// YAML
namespace YAML {
Node convert<csapex::connection_types::EndOfSequenceMessage>::encode(const csapex::connection_types::EndOfSequenceMessage& rhs) {
    return convert<csapex::connection_types::Message>::encode(rhs);
}

bool convert<csapex::connection_types::EndOfSequenceMessage>::decode(const Node& node, csapex::connection_types::EndOfSequenceMessage& rhs) {
    if(!node.IsMap()) {
        return false;
    }
    convert<csapex::connection_types::Message>::decode(node, rhs);
    return true;
}
}
