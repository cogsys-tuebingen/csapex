#ifndef FEEDBACK_H
#define FEEDBACK_H

/// PROJECT
#include <csapex/serialization/serializable.h>
#include <csapex/core/csapex_core.h>

/// SYSTEM
#include <string>

namespace csapex
{

class Feedback : public Serializable
{
public:
    Feedback(const std::string& message, uint8_t request_id);
    Feedback(const std::string& message);

    static const uint8_t PACKET_TYPE_ID = 6;

    virtual uint8_t getPacketType() const override;

    std::string getMessage() const;

    virtual void serialize(SerializationBuffer &data) const override;
    virtual void deserialize(SerializationBuffer& data) override;

    uint8_t getRequestID() const;

protected:
    virtual std::shared_ptr<Clonable> makeEmptyClone() const;

private:
    std::string message_;
    uint8_t request_id_;
};

}

#endif // FEEDBACK_H
