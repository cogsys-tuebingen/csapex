#ifndef FEEDBACK_H
#define FEEDBACK_H

/// PROJECT
#include <csapex/io/response.h>
#include <csapex/core/csapex_core.h>

/// SYSTEM
#include <string>

namespace csapex
{

class Feedback : public Response
{
public:
    Feedback(const std::string& message, uint8_t request_id);
    Feedback(const std::string& message);

    static const uint8_t PACKET_TYPE_ID = 6;

    virtual uint8_t getPacketType() const override;
    virtual std::string getType() const;

    std::string getMessage() const;

    virtual void serialize(SerializationBuffer &data) const override;
    virtual void deserialize(const SerializationBuffer& data) override;

protected:
    virtual std::shared_ptr<Clonable> makeEmptyClone() const;

private:
    std::string message_;
};

}

#endif // FEEDBACK_H
