/// HEADAER
#include <csapex/msg/message.h>

/// PROJECT
#include <csapex/utility/assert.h>
#include <csapex/utility/register_msg.h>

using namespace csapex;
using namespace connection_types;

Message::Version::Version(int major, int minor, int patch)
    : major_v(major),
      minor_v(minor),
      patch_v(patch)
{}

bool Message::Version::operator < (const Version& other)
{
    if(major_v < other.major_v) {
        return true;
    } else if(major_v > other.major_v) {
        return false;
    }
    // major equal
    if(minor_v < other.minor_v) {
        return true;
    } else if(minor_v > other.minor_v) {
        return false;
    }
    // major and minor equal
    return patch_v < other.patch_v;
}

bool Message::Version::operator == (const Version& other)
{
    return major_v == other.major_v &&
            minor_v == other.minor_v &&
            patch_v == other.patch_v;
}


bool Message::Version::operator > (const Version& other)
{
    return !(operator < (other)) && !(operator == (other));
}

bool Message::Version::valid() const
{
    return major_v >= 0;
}

Message::Version::operator bool() const
{
    return valid();
}



Message::Message(const std::string& name, const std::string &frame_id, Stamp stamp)
    : TokenData(name), frame_id(frame_id), stamp_micro_seconds(stamp)
{
    if(frame_id.size() > 0 && frame_id.at(0) == '/') {
        this->frame_id = frame_id.substr(1);
    }
}

Message::~Message()
{

}

void Message::cloneDataFrom(const Clonable& other)
{
    const Message& rhs = dynamic_cast<const Message&>(other);
    frame_id = rhs.frame_id;
    stamp_micro_seconds = rhs.stamp_micro_seconds;
}

/// YAML
namespace YAML {
Node convert<csapex::connection_types::Message>::encode(const csapex::connection_types::Message& rhs, const Message::Version &version) {
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

Message::Version convert<csapex::connection_types::Message>::decode(const Node& node, csapex::connection_types::Message& rhs) {
    if(!node.IsMap()) {
        return {};
    }
    rhs.frame_id = node["frame_id"].as<std::string>();
    rhs.stamp_micro_seconds = node["stamp"].as<std::int64_t>();

    YAML::Node vnode = node["version"];
    if(vnode.IsDefined()) {
        return Message::Version(vnode["major"].as<int>(), vnode["minor"].as<int>(), vnode["patch"].as<int>());
    } else {
        return {};
    }
}
}
