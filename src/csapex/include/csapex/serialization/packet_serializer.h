#ifndef PACKET_SERIALIZER_H
#define PACKET_SERIALIZER_H

/// PROJECT
#include <csapex/io/io_fwd.h>
#include <csapex/utility/singleton.hpp>
#include <csapex/serialization/serialization_fwd.h>

/// SYSTEM
#include <vector>
#include <map>

namespace csapex
{

class Serializer
{
public:
    virtual ~Serializer();

    virtual void serialize(const StreamableConstPtr& packet, SerializationBuffer &data) = 0;
    virtual StreamablePtr deserialize(const SerializationBuffer& data) = 0;
};

/**
 * @brief The Serializer interface has to be implemented for packet types
 */

/**
 * @brief The PacketSerializer class is the facade to all existing packet serializers.
 *        It is responsible for (de-)serializing any Packet.
 */
class PacketSerializer : public Singleton<PacketSerializer>, public Serializer
{
    friend class Singleton<PacketSerializer>;

public:
    static SerializationBuffer serializePacket(const StreamableConstPtr& packet);
    static StreamablePtr deserializePacket(SerializationBuffer &serial);
    static void registerSerializer(uint8_t type, Serializer* serializer);

public:
    void serialize(const StreamableConstPtr &packet, SerializationBuffer &data) override;
    StreamablePtr deserialize(const SerializationBuffer &data) override;


private:
    std::map<uint8_t, Serializer*> serializers_;
};

/**
 * @brief The SerializerRegistered template is used to register implementations of Serializer
 */
template <typename S>
struct SerializerRegistered
{
    SerializerRegistered(uint8_t type, S* instance) {
        PacketSerializer::registerSerializer(type, instance);
    }
};

}




#define CREATE_DEFAULT_SERIALIZER(Name) \
class Name##Serializer : public Singleton<Name##Serializer>, public Serializer \
{ \
public: \
    void serialize(const StreamableConstPtr& packet, SerializationBuffer &data) override \
    { \
        if(const std::shared_ptr<Name const>& res = std::dynamic_pointer_cast<Name const>(packet)) { \
            res->serialize(data); \
        } \
    } \
    StreamablePtr deserialize(const SerializationBuffer &data) override \
    { \
        std::shared_ptr<Name> res = Name::makeEmpty(); \
        res->deserialize(data); \
        return res; \
    } \
}; \
SerializerRegistered<Name##Serializer> g_CSAPEX_REGISTER_##NAME##_SERIALIZER_(Name::PACKET_TYPE_ID, &Name##Serializer::instance())

#endif // PACKET_SERIALIZER_H
