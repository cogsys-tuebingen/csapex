#ifndef RESPONSE_H
#define RESPONSE_H

/// PROJECT
#include <csapex/serialization/streamable.h>

/// SYSTEM
#include <string>

namespace csapex
{

class Response : public Streamable
{
public:
    Response(uint8_t id);

    static const uint8_t PACKET_TYPE_ID = 3;

    virtual uint8_t getPacketType() const override;
    virtual std::string getType() const = 0;

    uint8_t getRequestID() const;

protected:
    uint8_t request_id_;
};

}

#endif // RESPONSE_H
