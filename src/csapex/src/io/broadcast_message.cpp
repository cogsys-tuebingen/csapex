/// HEADER
#include <csapex/io/broadcast_message.h>

using namespace csapex;

uint8_t BroadcastMessage::getPacketType() const
{
    return PACKET_TYPE_ID;
}
