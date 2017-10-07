#ifndef BROADCAST_MESSAGE_SERIALIZER_H
#define BROADCAST_MESSAGE_SERIALIZER_H

/// PROJECT
#include <csapex/serialization/packet_serializer.h>
#include <csapex/io/broadcast_message.h>

/// SYSTEM
#include <inttypes.h>

namespace csapex
{

class BroadcastMessageSerializerInterface
{
public:
    virtual ~BroadcastMessageSerializerInterface();

    virtual void serialize(const BroadcastMessageConstPtr& packet, SerializationBuffer &data) = 0;
    virtual BroadcastMessagePtr deserialize(SerializationBuffer& data) = 0;
};


class BroadcastMessageSerializer : public Singleton<BroadcastMessageSerializer>, public Serializer
{
public:
    void serialize(const StreamableConstPtr& packet, SerializationBuffer &data) override;
    StreamablePtr deserialize(SerializationBuffer &data) override;

    static void registerSerializer(const std::string& type, std::shared_ptr<BroadcastMessageSerializerInterface> serializer);

private:
    std::map<std::string, std::shared_ptr<BroadcastMessageSerializerInterface>> serializers_;
};


/// REGISTRATION

template <typename S>
struct BroadcastMessageSerializerRegistered
{
    BroadcastMessageSerializerRegistered(const std::string& type) {
        BroadcastMessageSerializer::registerSerializer(type, std::make_shared<S>());
    }
};
}


#define CSAPEX_REGISTER_BROADCAST_SERIALIZER(Name) \
    namespace csapex \
    { \
    namespace io \
    { \
    class Name##Serializer : public BroadcastMessageSerializerInterface \
    { \
        virtual void serialize(const BroadcastMessageConstPtr& packet, SerializationBuffer &data) override \
        { \
            packet->serialize(data); \
        } \
        virtual BroadcastMessagePtr deserialize(SerializationBuffer& data) override \
        { \
            auto result = std::make_shared<Name>(); \
            result->deserialize(data); \
            return result; \
        } \
    }; \
    } \
    BroadcastMessageSerializerRegistered<io::Name##Serializer> g_register_broadcast_message_##Name##_(Name::typeName()); \
    }


#endif // BROADCAST_MESSAGE_SERIALIZER_H
