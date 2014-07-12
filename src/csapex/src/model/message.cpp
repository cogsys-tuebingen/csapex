/// HEADAER
#include <csapex/model/message.h>

/// PROJECT
#include <csapex/utility/assert.h>

using namespace csapex;
using namespace connection_types;

/***
 * MESSAGE
 */
Message::Message(const std::string& name, const std::string &frame_id)
    : ConnectionType(name), frame_id(frame_id)
{
    apex_assert(!frame_id.empty());
}

Message::~Message()
{

}

void Message::writeYaml(YAML::Emitter&) const
{

}
void Message::readYaml(const YAML::Node&)
{

}

/***
 * ANYMESSAGE
 */
AnyMessage::AnyMessage()
    : Message("anything", "/")
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

/***
 * NOMESSAGE
 */
NoMessage::NoMessage()
    : Message("nothing", "/")
{}

ConnectionType::Ptr NoMessage::clone()
{
    NoMessage::Ptr new_msg(new NoMessage);
    return new_msg;
}

ConnectionType::Ptr NoMessage::toType()
{
    Ptr new_msg(new NoMessage);
    return new_msg;
}

ConnectionType::Ptr NoMessage::make()
{
    Ptr new_msg(new NoMessage);
    return new_msg;
}

bool NoMessage::canConnectTo(const ConnectionType*) const
{
    return true;
}

bool NoMessage::acceptsConnectionFrom(const ConnectionType*) const
{
    return true;
}
