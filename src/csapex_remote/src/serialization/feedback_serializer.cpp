/// HEADER
#include <csapex/serialization/feedback_serializer.h>

/// PROJECT
#include <csapex/serialization/packet_serializer.h>
#include <csapex/utility/assert.h>
#include <csapex/serialization/io/std_io.h>
#include <csapex/io/feedback.h>

/// SYSTEM
#include <iostream>

using namespace csapex;

SerializerRegistered<FeedbackSerializer> g_register_Feedback_serializer_(Feedback::PACKET_TYPE_ID, &FeedbackSerializer::instance());

void FeedbackSerializer::serialize(const Streamable& packet, SerializationBuffer& data)
{
    packet.serializeVersioned(data);
}

StreamablePtr FeedbackSerializer::deserialize(const SerializationBuffer& data)
{
    FeedbackPtr res = std::make_shared<Feedback>("");
    res->deserializeVersioned(data);
    return res;
}
