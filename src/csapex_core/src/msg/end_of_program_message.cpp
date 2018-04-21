/// HEADER
#include <csapex/msg/end_of_program_message.h>

using namespace csapex;
using namespace connection_types;

EndOfProgramMessage::EndOfProgramMessage()
    : EndOfSequenceMessage(type<EndOfProgramMessage>::name())
{}
