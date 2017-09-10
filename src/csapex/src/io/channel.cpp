/// HEADER
#include <csapex/io/channel.h>

using namespace csapex;
using namespace csapex::io;

Channel::Channel(Session& session, const AUUID &name)
    : session_(session), name_(name)
{
    observe(session.raw_packet_received(name), raw_packet_received);
}
