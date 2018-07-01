/// HEADER
#include <csapex/msg/no_message.h>

/// PROJECT
#include <csapex/utility/register_msg.h>

CSAPEX_REGISTER_MESSAGE(csapex::connection_types::NoMessage)

using namespace csapex;
using namespace connection_types;

NoMessage::NoMessage() : MarkerMessage(type<NoMessage>::name(), 0)
{
}

void NoMessage::serialize(SerializationBuffer& data, SemanticVersion& version) const
{
}
void NoMessage::deserialize(const SerializationBuffer& data, const SemanticVersion& version)
{
}

/// YAML
namespace YAML
{
Node convert<csapex::connection_types::NoMessage>::encode(const csapex::connection_types::NoMessage& rhs)
{
    return convert<csapex::connection_types::Message>::encode(rhs);
}

bool convert<csapex::connection_types::NoMessage>::decode(const Node& node, csapex::connection_types::NoMessage& rhs)
{
    if (!node.IsMap()) {
        return false;
    }
    convert<csapex::connection_types::Message>::decode(node, rhs);
    return true;
}
}  // namespace YAML
