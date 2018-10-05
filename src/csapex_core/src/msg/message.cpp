/// HEADAER
#include <csapex/msg/message.h>

/// PROJECT
#include <csapex/utility/assert.h>
#include <csapex/utility/register_msg.h>

using namespace csapex;
using namespace connection_types;

Message::Message(const std::string& name, const std::string& frame_id, Stamp stamp) : TokenData(name), frame_id(frame_id), stamp_micro_seconds(stamp)
{
    if (frame_id.size() > 0 && frame_id.at(0) == '/') {
        this->frame_id = frame_id.substr(1);
    }
}

Message::~Message()
{
}

bool Message::cloneDataFrom(const Clonable& other)
{
    const Message& rhs = dynamic_cast<const Message&>(other);
    frame_id = rhs.frame_id;
    stamp_micro_seconds = rhs.stamp_micro_seconds;

    return true;
}

void Message::serialize(SerializationBuffer& data, SemanticVersion& version) const
{
    TokenData::serialize(data, version);

    data << frame_id;
    data << stamp_micro_seconds;
}
void Message::deserialize(const SerializationBuffer& data, const SemanticVersion& version)
{
    TokenData::deserialize(data, version);

    data >> frame_id;
    data >> stamp_micro_seconds;
}

/// YAML
namespace YAML
{
Node convert<csapex::connection_types::Message>::encode(const csapex::connection_types::Message& rhs, const SemanticVersion& version)
{
    Node vnode;
    vnode["major"] = version.major_v;
    vnode["minor"] = version.minor_v;
    vnode["patch"] = version.patch_v;

    Node node;
    node["version"] = vnode;
    node["frame_id"] = rhs.frame_id;
    node["stamp"] = rhs.stamp_micro_seconds;
    return node;
}

SemanticVersion convert<csapex::connection_types::Message>::decode(const Node& node, csapex::connection_types::Message& rhs)
{
    if (!node.IsMap()) {
        return {};
    }
    rhs.frame_id = node["frame_id"].as<std::string>();
    rhs.stamp_micro_seconds = node["stamp"].as<std::int64_t>();

    YAML::Node vnode = node["version"];
    if (vnode.IsDefined()) {
        return SemanticVersion(vnode["major"].as<int>(), vnode["minor"].as<int>(), vnode["patch"].as<int>());
    } else {
        return {};
    }
}
}  // namespace YAML
