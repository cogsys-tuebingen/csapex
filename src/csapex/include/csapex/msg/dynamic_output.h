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
    DynamicOutput(OutputTransition* transition, const UUID &uuid);
    DynamicOutput(OutputTransition* transition, Unique *parent, int sub_id);

    virtual void publish(ConnectionType::ConstPtr message) override;

    virtual void commitMessages() override;
    virtual bool hasMessage() override;
    virtual void nextMessage() override;
    virtual ConnectionTypeConstPtr getMessage() const override;

    virtual void clear() override;

private:
    std::deque<ConnectionType::ConstPtr> messages_to_send_;
    std::deque<ConnectionType::ConstPtr> committed_messages_;
    ConnectionTypeConstPtr current_message_;
};
}

#endif // DYNAMIC_OUTPUT_H

