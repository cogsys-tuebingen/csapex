/// HEADER
#include <csapex_boolean/boolean_message.h>

using namespace csapex;
using namespace connection_types;


BooleanMessage::BooleanMessage()
    : MessageTemplate<bool, BooleanMessage> ("bool")
{}
