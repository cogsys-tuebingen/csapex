/// HEADER
#include <csapex/msg/no_message.h>

using namespace csapex;
using namespace connection_types;

NoMessage::NoMessage()
    : MarkerMessage(type<NoMessage>::name(), 0)
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

