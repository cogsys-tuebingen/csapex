/// HEADER
#include <csapex/msg/marker_message.h>

using namespace csapex;
using namespace connection_types;

MarkerMessage::MarkerMessage(const std::string& name, Stamp stamp)
    : Message(name, "/", stamp)
{}

bool MarkerMessage::canConnectTo(const TokenData*) const
{
    return true;
}

bool MarkerMessage::acceptsConnectionFrom(const TokenData*) const
{
    return true;
}
