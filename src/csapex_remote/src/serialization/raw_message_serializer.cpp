/// HEADER
#include <csapex/serialization/raw_message_serializer.h>

/// PROJECT
#include <csapex/serialization/packet_serializer.h>
#include <csapex/utility/assert.h>
#include <csapex/serialization/io/std_io.h>
#include <csapex/io/raw_message.h>

/// SYSTEM
#include <iostream>

using namespace csapex;

SerializerRegistered<RawMessageSerializer> g_register_RawMessage_serializer_(RawMessage::PACKET_TYPE_ID, &RawMessageSerializer::instance());


void RawMessageSerializer::serialize(const Streamable& packet, SerializationBuffer& data)
{
    packet.serializeVersioned(data);
}

StreamablePtr RawMessageSerializer::deserialize(const SerializationBuffer& data)
{
    RawMessagePtr res = std::make_shared<RawMessage>();
    res->deserializeVersioned(data);
    return res;
}
