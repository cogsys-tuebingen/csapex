/// HEADER
#include <csapex/msg/end_of_sequence_message.h>

using namespace csapex;
using namespace connection_types;

EndOfSequenceMessage::EndOfSequenceMessage()
    : MarkerMessage(type<EndOfSequenceMessage>::name(), 0)
{}

EndOfSequenceMessage::EndOfSequenceMessage(const std::string& name)
    : MarkerMessage(name, 0)
{}


void EndOfSequenceMessage::serialize(SerializationBuffer &data) const
{
}
void EndOfSequenceMessage::deserialize(const SerializationBuffer& data)
{
}
