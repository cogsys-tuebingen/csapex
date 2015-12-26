/// HEADER
#include <csapex/signal/signal.h>

/// SYSTEM
#include <csapex/utility/register_msg.h>
#include <yaml-cpp/yaml.h>

using namespace csapex;
using namespace connection_types;

Signal::Signal()
    : ConnectionType("Signal")
{
}

ConnectionType::Ptr Signal::clone() const
{
    Ptr new_msg(new Signal);
    return new_msg;
}

ConnectionType::Ptr Signal::toType() const
{
    Ptr new_msg(new Signal);
    return new_msg;
}

bool Signal::acceptsConnectionFrom(const ConnectionType* other_side) const
{
    return dynamic_cast<const Signal*> (other_side);
}

/// YAML
namespace YAML {
Node convert<csapex::connection_types::Signal>::encode(const csapex::connection_types::Signal&) {
    Node node;
    return node;
}
bool convert<csapex::connection_types::Signal>::decode(const Node&, csapex::connection_types::Signal&) {
    return true;
}
}
