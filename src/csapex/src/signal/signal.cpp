/// HEADER
#include <csapex/signal/signal.h>

/// SYSTEM
#include <csapex/utility/register_msg.h>

using namespace csapex;
using namespace connection_types;

Signal::Signal()
    : ConnectionType("Signal")
{
}

ConnectionType::Ptr Signal::clone() {
    Ptr new_msg(new Signal);
    return new_msg;
}

ConnectionType::Ptr Signal::toType() {
    Ptr new_msg(new Signal);
    return new_msg;
}

bool Signal::acceptsConnectionFrom(const ConnectionType* other_side) const {
    return dynamic_cast<const Signal*> (other_side);
}
