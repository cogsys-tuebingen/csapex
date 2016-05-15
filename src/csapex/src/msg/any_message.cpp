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

TokenData::Ptr AnyMessage::clone() const
{
    AnyMessage::Ptr new_msg(new AnyMessage);
    return new_msg;
}

TokenData::Ptr AnyMessage::toType() const
{
    Ptr new_msg(new AnyMessage);
    return new_msg;
}

bool AnyMessage::canConnectTo(const TokenData*) const
{
    return true;
}

bool AnyMessage::acceptsConnectionFrom(const TokenData*) const
{
    return true;
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
