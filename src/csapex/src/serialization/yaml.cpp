/// HEADER
#include <csapex/serialization/yaml.h>

/// PROJECT
#include <csapex/msg/message_factory.h>

using namespace csapex;

namespace YAML {
Node convert<csapex::ConnectionType>::encode(const csapex::ConnectionType& rhs)
{
    return MessageFactory::serializeMessage(rhs);
}

bool convert<csapex::ConnectionType>::decode(const Node& node, csapex::ConnectionType& rhs)
{
    rhs = *MessageFactory::deserializeMessage(node);
    return true;
}
}
