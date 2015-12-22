/// HEADAER
#include <csapex/msg/message.h>

/// PROJECT
#include <csapex/utility/assert.h>
#include <csapex/utility/register_msg.h>

using namespace csapex;
using namespace connection_types;

Message::Message(const std::string& name, const std::string &frame_id, Stamp stamp)
    : ConnectionType(name), frame_id(frame_id), stamp_micro_seconds(stamp)
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
    node["stamp"] = rhs.stamp_micro_seconds;
    return node;
}

bool convert<csapex::connection_types::Message>::decode(const Node& node, csapex::connection_types::Message& rhs) {
    if(!node.IsMap()) {
        return false;
    }
    rhs.frame_id = node["frame_id"].as<std::string>();
    rhs.stamp_micro_seconds = node["stamp"].as<u_int64_t>();
    return true;
}
}
