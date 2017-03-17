#ifndef PACKET_H
#define PACKET_H

/// PROJECT
#include <csapex/serialization/serialization_buffer.h>

/// SYSTEM
#include <inttypes.h>
#include <vector>

namespace csapex
{

class Serializable
{
public:
    virtual ~Serializable();

    virtual uint8_t getPacketType() const = 0;

    virtual void serialize(SerializationBuffer &data) = 0;
    virtual void deserialize(SerializationBuffer& data) = 0;
};

}

#endif
