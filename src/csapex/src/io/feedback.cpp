/// HEADER
#include <csapex/io/feedback.h>

/// SYSTEM
#include <csapex/serialization/serialization_buffer.h>

using namespace csapex;

uint8_t Feedback::getPacketType() const
{
    return PACKET_TYPE_ID;
}

Feedback::Feedback(const std::string &message)
    : message_(message), request_id_(0)
{

}

Feedback::Feedback(const std::string &message, uint8_t request_id)
    : message_(message), request_id_(request_id)
{

}

std::string Feedback::getMessage() const
{
    return message_;
}



void Feedback::serialize(SerializationBuffer &data) const
{
    data << message_;
    data << request_id_;
}
void Feedback::deserialize(SerializationBuffer& data)
{
    data >> message_;
    data >> request_id_;
}

std::shared_ptr<Clonable> Feedback::makeEmptyClone() const
{
    return std::make_shared<Feedback>("");
}

uint8_t Feedback::getRequestID() const
{
    return request_id_;
}

