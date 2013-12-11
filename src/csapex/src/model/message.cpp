/// HEADAER
#include <csapex/model/message.h>

using namespace csapex;
using namespace connection_types;

Message::Message(const std::string& name)
    : ConnectionType(name)
{
}

Message::~Message()
{

}

void Message::writeYaml(YAML::Emitter&)
{

}
void Message::readYaml(const YAML::Node&)
{

}


AnyMessage::AnyMessage()
    : Message("anything")
{}

ConnectionType::Ptr AnyMessage::clone()
{
    AnyMessage::Ptr new_msg(new AnyMessage);
    return new_msg;
}

ConnectionType::Ptr AnyMessage::toType()
{
    Ptr new_msg(new AnyMessage);
    return new_msg;
}

ConnectionType::Ptr AnyMessage::make()
{
    Ptr new_msg(new AnyMessage);
    return new_msg;
}

bool AnyMessage::canConnectTo(const ConnectionType*) const
{
    return true;
}

bool AnyMessage::acceptsConnectionFrom(const ConnectionType*) const
{
    return true;
}
