#ifndef STATIC_OUTPUT_H
#define STATIC_OUTPUT_H

/// COMPONENT
#include <csapex/msg/output.h>

namespace csapex
{
class StaticOutput : public Output
{
public:
    StaticOutput(const UUID &uuid);
    StaticOutput(Unique *parent, int sub_id);

    virtual void publish(ConnectionType::ConstPtr message) override;

    virtual bool sendMessages() override;
    virtual bool hasMessage() override;

    virtual void disable() override;
    virtual void reset() override;
    virtual void clear() override;

    ConnectionType::ConstPtr getMessage();

private:
    ConnectionType::ConstPtr message_;
    ConnectionType::ConstPtr message_to_send_;
};
}

#endif // STATIC_OUTPUT_H

