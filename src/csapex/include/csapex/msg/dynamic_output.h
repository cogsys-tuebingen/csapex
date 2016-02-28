#ifndef DYNAMIC_OUTPUT_H
#define DYNAMIC_OUTPUT_H

/// COMPONENT
#include <csapex/msg/output.h>

/// SYSTEM
#include <deque>

namespace csapex
{
class DynamicInput;

class DynamicOutput : public Output
{
public:
    DynamicOutput(const UUID &uuid);

    void clearCorrespondents();
    void addCorrespondent(DynamicInput* input);

    virtual void addMessage(ConnectionType::ConstPtr message) override;

    virtual void setMultipart(bool multipart, bool last_part) override;

    virtual void commitMessages() override;
    virtual bool hasMessage() override;
    virtual bool hasMarkerMessage() override;
    virtual void nextMessage() override;
    virtual ConnectionTypeConstPtr getMessage() const override;

    virtual void startReceiving() override;

private:
    std::vector<DynamicInput*> correspondents_;

    std::deque<ConnectionType::ConstPtr> messages_to_send_;
    std::deque<ConnectionType::ConstPtr> committed_messages_;
    ConnectionTypeConstPtr current_message_;
};
}

#endif // DYNAMIC_OUTPUT_H

