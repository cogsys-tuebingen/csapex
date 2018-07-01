/// HEADER
#include <csapex/io/request.h>

using namespace csapex;

uint8_t Request::getPacketType() const
{
    return PACKET_TYPE_ID;
}

Request::Request(uint8_t id) : request_id_(id)
{
}

void Request::overwriteRequestID(uint8_t id) const
{
    request_id_ = id;
}

uint8_t Request::getRequestID() const
{
    return request_id_;
}
