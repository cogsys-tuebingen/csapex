/// HEADER
#include <csapex/serialization/yaml.h>

/// PROJECT
#include <csapex/serialization/message_serializer.h>

using namespace csapex;

namespace YAML {
Node convert<csapex::Token>::encode(const csapex::Token& rhs)
{
    return MessageSerializer::serializeMessage(rhs);
}

bool convert<csapex::Token>::decode(const Node& node, csapex::Token& rhs)
{
    rhs = *MessageSerializer::deserializeMessage(node);
    return true;
}

Node convert<csapex::TokenConstPtr>::encode(const csapex::TokenConstPtr& rhs)
{
    return MessageSerializer::serializeMessage(*rhs);
}

bool convert<csapex::TokenConstPtr>::decode(const Node& node, csapex::TokenConstPtr& rhs)
{
    TokenConstPtr ptr = MessageSerializer::deserializeMessage(node);
    rhs.swap(ptr);
    return true;
}
}
