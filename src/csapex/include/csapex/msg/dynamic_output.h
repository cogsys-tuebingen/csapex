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

    virtual void addMessage(TokenPtr message) override;

    virtual void setMultipart(bool multipart, bool last_part) override;

    virtual bool commitMessages(bool is_activated) override;
    virtual bool hasMessage() override;
    virtual bool hasMarkerMessage() override;
    virtual void nextMessage() override;
    virtual TokenPtr getToken() const override;

    virtual void clearBuffer() override;

private:
    std::vector<DynamicInput*> correspondents_;

    std::deque<TokenPtr> messages_to_send_;
    std::deque<TokenPtr> committed_messages_;
    TokenPtr current_message_;
};
}

#endif // DYNAMIC_OUTPUT_H

