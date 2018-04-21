/// HEADER
#include <csapex/msg/no_message.h>

using namespace csapex;
using namespace connection_types;

NoMessage::NoMessage()
    : MarkerMessage(type<NoMessage>::name(), 0)
{}

void NoMessage::serialize(SerializationBuffer &data) const
{
}
void NoMessage::deserialize(const SerializationBuffer& data)
{
}
