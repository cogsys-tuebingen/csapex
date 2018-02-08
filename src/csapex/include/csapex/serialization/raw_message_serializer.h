#ifndef RAW_MESSAGE_SERIALIZER_H
#define RAW_MESSAGE_SERIALIZER_H

/// PROJECT
#include <csapex/serialization/packet_serializer.h>
#include <csapex/io/raw_message.h>

/// SYSTEM
#include <inttypes.h>

namespace csapex
{

class RawMessageSerializer : public Singleton<RawMessageSerializer>, public Serializer
{
public:
    void serialize(const Streamable& packet, SerializationBuffer &data) override;
    StreamablePtr deserialize(const SerializationBuffer &data) override;
};

}
#endif // RAW_MESSAGE_SERIALIZER_H
