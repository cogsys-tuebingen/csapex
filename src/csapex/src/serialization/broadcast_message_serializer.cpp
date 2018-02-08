/// HEADER
#include <csapex/serialization/broadcast_message_serializer.h>

/// PROJECT
#include <csapex/serialization/packet_serializer.h>
#include <csapex/utility/assert.h>
#include <csapex/serialization/serialization_buffer.h>

/// SYSTEM
#include <iostream>

using namespace csapex;

SerializerRegistered<BroadcastMessageSerializer> g_register_BroadcastMessage_serializer_(BroadcastMessage::PACKET_TYPE_ID, &BroadcastMessageSerializer::instance());


BroadcastMessageSerializerInterface::~BroadcastMessageSerializerInterface()
{

}

void BroadcastMessageSerializer::serialize(const Streamable& packet, SerializationBuffer& data)
{
    if(const BroadcastMessage* broadcast = dynamic_cast<const BroadcastMessage*>(&packet)) {
//        std::cerr << "serializing BroadcastMessage" << std::endl;
        std::string type = broadcast->getType();
        auto it = serializers_.find(type);
        if(it != serializers_.end()) {

            data << type;

            // defer serialization to the corresponding serializer
            std::shared_ptr<BroadcastMessageSerializerInterface> serializer = it->second;
            serializer->serialize(*broadcast, data);

        } else {
            std::cerr << "cannot serialize BroadcastMessage of type " << type << ", none of the " << serializers_.size() << " serializers matches." << std::endl;
        }

    }
}

StreamablePtr BroadcastMessageSerializer::deserialize(const SerializationBuffer& data)
{
//    std::cerr << "deserializing BroadcastMessage" << std::endl;

    std::string type;
    data >> type;

    auto it = serializers_.find(type);
    if(it != serializers_.end()) {

//        std::cerr << "deserializing BroadcastMessage (type=" << type << ")" << std::endl;

        // defer serialization to the corresponding serializer
        std::shared_ptr<BroadcastMessageSerializerInterface> serializer = it->second;
        return serializer->deserialize(data);

    } else {
        std::cerr << "cannot deserialize BroadcastMessage of type " << type << ", none of the " << serializers_.size() << " serializers matches." << std::endl;
    }


    return BroadcastMessagePtr();
}

void BroadcastMessageSerializer::registerSerializer(const std::string &type, std::shared_ptr<BroadcastMessageSerializerInterface> serializer)
{
//    std::cout << "registering serializer of type " << type << std::endl;
    instance().serializers_[type] = serializer;
}
