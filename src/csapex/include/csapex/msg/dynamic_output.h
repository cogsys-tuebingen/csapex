#ifndef DYNAMIC_OUTPUT_H
#define DYNAMIC_OUTPUT_H

/// COMPONENT
#include <csapex/msg/output.h>

/// SYSTEM
#include <deque>

namespace csapex
{
class DynamicOutput : public Output
{
public:
    DynamicOutput(const UUID &uuid);
    DynamicOutput(Unique *parent, int sub_id);

    virtual void publish(ConnectionType::ConstPtr message) override;

    virtual bool sendMessages() override;
    virtual bool hasMessage() override;

    virtual void clear() override;

private:
    std::deque<ConnectionType::ConstPtr> messages_to_send_;
};
}

#endif // DYNAMIC_OUTPUT_H

