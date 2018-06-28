/// HEADER
#include <csapex/msg/any_message.h>

/// PROJECT
#include <csapex/utility/register_msg.h>

CSAPEX_REGISTER_MESSAGE(csapex::connection_types::AnyMessage)

using namespace csapex;
using namespace connection_types;

AnyMessage::AnyMessage()
    : Message(type<AnyMessage>::name(), "/", 0)
{}

bool AnyMessage::canConnectTo(const TokenData*) const
{
    return true;
}

bool AnyMessage::acceptsConnectionFrom(const TokenData*) const
{
    return true;
}

void AnyMessage::serialize(SerializationBuffer &data, SemanticVersion& version) const
{
}
void AnyMessage::deserialize(const SerializationBuffer& data, const SemanticVersion &version)
{
}

/// YAML
namespace YAML {
Node convert<csapex::connection_types::AnyMessage>::encode(const csapex::connection_types::AnyMessage& rhs) {
    return convert<csapex::connection_types::Message>::encode(rhs);
}

bool convert<csapex::connection_types::AnyMessage>::decode(const Node& node, csapex::connection_types::AnyMessage& rhs) {
    if(!node.IsMap()) {
        return false;
    }
    convert<csapex::connection_types::Message>::decode(node, rhs);
    return true;
}
}
