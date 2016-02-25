/// HEADER
#include <csapex/msg/end_of_sequence_message.h>

using namespace csapex;
using namespace connection_types;

EndOfSequenceMessage::EndOfSequenceMessage()
    : MarkerMessage(type<EndOfSequenceMessage>::name(), 0)
{}

ConnectionType::Ptr EndOfSequenceMessage::clone() const
{
    EndOfSequenceMessage::Ptr new_msg(new EndOfSequenceMessage);
    return new_msg;
}

ConnectionType::Ptr EndOfSequenceMessage::toType() const
{
    Ptr new_msg(new EndOfSequenceMessage);
    return new_msg;
}
