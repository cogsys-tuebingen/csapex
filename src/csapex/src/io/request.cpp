/// HEADER
#include <csapex/io/request.h>

using namespace csapex;

uint8_t Request::getPacketType() const
{
    return PACKET_TYPE_ID;
}

Request::Request()
{

}

