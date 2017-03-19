#ifndef REQUEST_H
#define REQUEST_H

/// PROJECT
#include <csapex/serialization/serializable.h>
#include <csapex/core/csapex_core.h>
#include <csapex/io/io_fwd.h>

/// SYSTEM
#include <string>

namespace csapex
{

class Request : public Serializable
{
public:
    Request();

    static const uint8_t PACKET_TYPE_ID = 2;

    virtual uint8_t getPacketType() const override;
    virtual std::string getType() const = 0;

    virtual ResponsePtr execute(CsApexCore& core) const = 0;
};

}

#endif // REQUEST_H
