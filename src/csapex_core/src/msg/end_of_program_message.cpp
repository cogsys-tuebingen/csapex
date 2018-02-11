/// HEADER
#include <csapex/msg/end_of_program_message.h>

using namespace csapex;
using namespace connection_types;

EndOfProgramMessage::EndOfProgramMessage()
    : EndOfSequenceMessage(type<EndOfProgramMessage>::name())
{}

TokenData::Ptr EndOfProgramMessage::clone() const
{
    EndOfProgramMessage::Ptr new_msg(new EndOfProgramMessage);
    return new_msg;
}

TokenData::Ptr EndOfProgramMessage::toType() const
{
    Ptr new_msg(new EndOfProgramMessage);
    return new_msg;
}
