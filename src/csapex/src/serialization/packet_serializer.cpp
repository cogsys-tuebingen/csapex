/// HEADER
#include <csapex/serialization/packet_serializer.h>

/// PROJECT
#include <csapex/utility/assert.h>
#include <csapex/serialization/serializable.h>

/// SYSTEM
#include <iostream>
#include <sstream>

using namespace csapex;

Serializer::~Serializer()
{

}
SerializationBuffer PacketSerializer::serializePacket(const SerializableConstPtr &packet)
{
    SerializationBuffer data;
    instance().serialize(packet, data);
    data.finalize();
    return data;
}

SerializablePtr PacketSerializer::deserializePacket(SerializationBuffer& serial)
{
    return instance().deserialize(serial);
}

void PacketSerializer::registerSerializer(uint8_t type, Serializer *serializer)
{
    instance().serializers_[type] = serializer;
}


void PacketSerializer::serialize(const SerializableConstPtr& packet, SerializationBuffer &data)
{
    // determine packet type
    uint8_t type = packet->getPacketType();
    auto it = serializers_.find(type);
    if(it != serializers_.end()) {
        data << type;

        // defer serialization to the corresponding serializer
        Serializer* serializer = it->second;
        serializer->serialize(packet, data);
    }
}

SerializablePtr PacketSerializer::deserialize(SerializationBuffer& data)
{
    // determine packet type
    uint8_t type;
    data >> type;

    auto it = serializers_.find(type);
    if(it != serializers_.end()) {
        // defer deserialization to the corresponding serializer
        Serializer* serializer = it->second;
        return serializer->deserialize(data);

    } else {
        std::cerr << "cannot deserialize packet of type: " << (int) type << std::endl;
    }

    return SerializablePtr();
}
