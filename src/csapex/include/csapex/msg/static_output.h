#ifndef STATIC_OUTPUT_H
#define STATIC_OUTPUT_H

/// COMPONENT
#include <csapex/msg/output.h>

namespace csapex
{
class StaticOutput : public Output
{
public:
    StaticOutput(OutputTransition* transition, const UUID &uuid);
    StaticOutput(OutputTransition* transition, Unique *parent, int sub_id);

    virtual void publish(ConnectionType::ConstPtr message) override;

    virtual void setMultipart(bool multipart, bool last_part) override;

    virtual void commitMessages() override;
    virtual bool hasMessage() override;
    virtual void nextMessage() override;
    virtual ConnectionTypeConstPtr getMessage() const override;

    virtual void disable() override;
    virtual void reset() override;
    virtual void clear() override;

    ConnectionType::ConstPtr getMessage();

private:
    ConnectionType::ConstPtr message_to_send_;
    ConnectionType::ConstPtr committed_message_;
};
}

#endif // STATIC_OUTPUT_H
