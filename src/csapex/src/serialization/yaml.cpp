/// HEADER
#include <csapex/serialization/yaml.h>

/// PROJECT
#include <csapex/serialization/message_serializer.h>

using namespace csapex;

namespace YAML {
Node convert<csapex::ConnectionType>::encode(const csapex::ConnectionType& rhs)
{
    return MessageSerializer::serializeMessage(rhs);
}

bool convert<csapex::ConnectionType>::decode(const Node& node, csapex::ConnectionType& rhs)
{
    rhs = *MessageSerializer::deserializeMessage(node);
    return true;
}
}
