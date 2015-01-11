/// HEADAER
#include <csapex/msg/message.h>

/// PROJECT
#include <csapex/utility/assert.h>
#include <csapex/utility/register_msg.h>

CSAPEX_REGISTER_MESSAGE(csapex::connection_types::AnyMessage)
//CSAPEX_REGISTER_MESSAGE(csapex::connection_types::NoMessage)

using namespace csapex;
using namespace connection_types;

/***
 * MESSAGE
 */
Message::Message(const std::string& name, const std::string &frame_id, Stamp stamp)
    : ConnectionType(name), frame_id(frame_id), stamp(stamp)
{
    apex_assert(!frame_id.empty());
}

Message::~Message()
{

}


/// YAML
namespace YAML {
Node convert<csapex::connection_types::Message>::encode(const csapex::connection_types::Message& rhs) {
    Node node;
    node["frame_id"] = rhs.frame_id;
    node["stamp"] = rhs.stamp;
    return node;
}

bool convert<csapex::connection_types::Message>::decode(const Node& node, csapex::connection_types::Message& rhs) {
    if(!node.IsMap()) {
        return false;
    }
    rhs.frame_id = node["frame_id"].as<std::string>();
    rhs.stamp = node["stamp"].as<u_int64_t>();
    return true;
}
}

/***
 * ANYMESSAGE
 */
AnyMessage::AnyMessage()
    : Message(type<AnyMessage>::name(), "/", 0)
{}

ConnectionType::Ptr AnyMessage::clone() const
{
    AnyMessage::Ptr new_msg(new AnyMessage);
    return new_msg;
}

ConnectionType::Ptr AnyMessage::toType() const
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



/// YAML
namespace YAML {
Node convert<csapex::connection_types::AnyMessage>::encode(const csapex::connection_types::AnyMessage& rhs) {
    return convert<csapex::connection_types::Message>::encode(rhs);
}

bool convert<csapex::connection_types::AnyMessage>::decode(const Node& node, csapex::connection_types::AnyMessage& rhs) {
    if(!node.IsMap()) {
        return false;
    }
    convert<csapex::connection_types::Message>::decode(node, rhs);
    return true;
}
}


/***
 * NOMESSAGE
 */
NoMessage::NoMessage()
    : Message(type<NoMessage>::name(), "/", 0)
{}

ConnectionType::Ptr NoMessage::clone() const
{
    NoMessage::Ptr new_msg(new NoMessage);
    return new_msg;
}

ConnectionType::Ptr NoMessage::toType() const
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
