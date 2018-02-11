#ifndef REQUEST_H
#define REQUEST_H

/// PROJECT
#include <csapex/serialization/streamable.h>
#include <csapex/core/csapex_core.h>
#include <csapex/io/remote_io_fwd.h>

/// SYSTEM
#include <string>

namespace csapex
{

class Request : public Streamable
{
public:
    Request(uint8_t id);

    static const uint8_t PACKET_TYPE_ID = 2;

    virtual uint8_t getPacketType() const override;
    virtual std::string getType() const = 0;

    virtual ResponsePtr execute(const SessionPtr& session, CsApexCore& core) const = 0;

    void overwriteRequestID(uint8_t id) const;
    uint8_t getRequestID() const;

private:
    mutable uint8_t request_id_;
};

}

#endif // REQUEST_H
