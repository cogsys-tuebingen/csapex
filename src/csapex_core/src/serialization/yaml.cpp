/// HEADER
#include <csapex/serialization/yaml.h>

/// PROJECT
#include <csapex/serialization/message_serializer.h>

using namespace csapex;

namespace YAML {
Node convert<csapex::TokenData>::encode(const csapex::TokenData& rhs)
{
    return MessageSerializer::serializeMessage(rhs);
}

bool convert<csapex::TokenData>::decode(const Node& node, csapex::TokenData& rhs)
{
    rhs = *MessageSerializer::deserializeMessage(node);
    return true;
}

Node convert<csapex::TokenDataConstPtr>::encode(const csapex::TokenDataConstPtr& rhs)
{
    return MessageSerializer::serializeMessage(*rhs);
}

bool convert<csapex::TokenDataConstPtr>::decode(const Node& node, csapex::TokenDataConstPtr& rhs)
{
    TokenDataConstPtr ptr = MessageSerializer::deserializeMessage(node);
    rhs.swap(ptr);
    return true;
}
}
