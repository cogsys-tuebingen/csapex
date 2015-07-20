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

Node convert<csapex::ConnectionTypeConstPtr>::encode(const csapex::ConnectionTypeConstPtr& rhs)
{
    return MessageSerializer::serializeMessage(*rhs);
}

bool convert<csapex::ConnectionTypeConstPtr>::decode(const Node& node, csapex::ConnectionTypeConstPtr& rhs)
{
    ConnectionTypeConstPtr ptr = MessageSerializer::deserializeMessage(node);
    rhs.swap(ptr);
    return true;
}
}
