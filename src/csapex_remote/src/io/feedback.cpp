/// HEADER
#include <csapex/io/feedback.h>

/// SYSTEM
#include <csapex/serialization/io/std_io.h>

using namespace csapex;

Feedback::Feedback(const std::string &message)
    : Response(0), message_(message)
{

}

Feedback::Feedback(const std::string &message, uint8_t request_id)
    : Response(request_id), message_(message)
{

}

uint8_t Feedback::getPacketType() const
{
    return PACKET_TYPE_ID;
}

std::string Feedback::getType() const
{
    return "Feedback";
}

std::string Feedback::getMessage() const
{
    return message_;
}


void Feedback::serialize(SerializationBuffer &data) const
{
    data << request_id_;
    data << message_;
}
void Feedback::deserialize(const SerializationBuffer& data)
{
    data >> request_id_;
    data >> message_;
}

std::shared_ptr<Clonable> Feedback::makeEmptyClone() const
{
    return std::make_shared<Feedback>("");
}

