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
TokenData::Ptr EndOfSequenceMessage::clone() const
{
    EndOfSequenceMessage::Ptr new_msg(new EndOfSequenceMessage);
    return new_msg;
}

TokenData::Ptr EndOfSequenceMessage::toType() const
{
    Ptr new_msg(new EndOfSequenceMessage);
    return new_msg;
}
