/// HEADER
#include <csapex/io/response.h>

using namespace csapex;

uint8_t Response::getPacketType() const
{
    return PACKET_TYPE_ID;
}

Response::Response()
{

}
