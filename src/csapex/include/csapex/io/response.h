#ifndef RESPONSE_H
#define RESPONSE_H

/// PROJECT
#include <csapex/serialization/serializable.h>
#include <csapex/core/csapex_core.h>

/// SYSTEM
#include <string>

namespace csapex
{

class Response : public Serializable
{
public:
    Response();

    static const uint8_t PACKET_TYPE_ID = 3;

    virtual uint8_t getPacketType() const override;
    virtual std::string getType() const = 0;
};

}

#endif // RESPONSE_H
