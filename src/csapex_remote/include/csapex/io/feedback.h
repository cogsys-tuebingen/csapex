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
protected:
    CLONABLE_IMPLEMENTATION(Feedback);

public:
    Feedback(const std::string& message, uint8_t request_id);
    Feedback(const std::string& message);

    static const uint8_t PACKET_TYPE_ID = 6;

    virtual uint8_t getPacketType() const override;
    virtual std::string getType() const;

    std::string getMessage() const;

    virtual void serialize(SerializationBuffer& data, SemanticVersion& version) const override;
    virtual void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;

private:
    Feedback();

private:
    std::string message_;
};

}  // namespace csapex

#endif  // FEEDBACK_H
