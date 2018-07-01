/// HEADER
#include <csapex/io/response.h>

using namespace csapex;

uint8_t Response::getPacketType() const
{
    return PACKET_TYPE_ID;
}

Response::Response(uint8_t id) : request_id_(id)
{
}

uint8_t Response::getRequestID() const
{
    return request_id_;
}
