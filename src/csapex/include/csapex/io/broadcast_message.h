#ifndef BROADCAST_MESSAGE_H
#define BROADCAST_MESSAGE_H

/// PROJECT
#include <csapex/serialization/streamable.h>

/// SYSTEM
#include <string>

namespace csapex
{

class BroadcastMessage : public Streamable
{
public:
    static const uint8_t PACKET_TYPE_ID = 5;

    virtual uint8_t getPacketType() const override;
    virtual std::string getType() const = 0;
};

}

#endif // BROADCAST_MESSAGE_H
