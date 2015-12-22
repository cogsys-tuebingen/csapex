/// HEADER
#include <csapex/msg/no_message.h>

using namespace csapex;
using namespace connection_types;

NoMessage::NoMessage()
    : Message(type<NoMessage>::name(), "/", 0)
{}

ConnectionType::Ptr NoMessage::clone() const
{
    NoMessage::Ptr new_msg(new NoMessage);
    return new_msg;
}

ConnectionType::Ptr NoMessage::toType() const
{
    Ptr new_msg(new NoMessage);
    return new_msg;
}

bool NoMessage::canConnectTo(const ConnectionType*) const
{
    return true;
}

bool NoMessage::acceptsConnectionFrom(const ConnectionType*) const
{
    return true;
}
