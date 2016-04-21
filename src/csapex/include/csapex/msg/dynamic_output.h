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

    virtual void addMessage(Token::ConstPtr message) override;

    virtual void setMultipart(bool multipart, bool last_part) override;

    virtual void commitMessages() override;
    virtual bool hasMessage() override;
    virtual bool hasMarkerMessage() override;
    virtual void nextMessage() override;
    virtual TokenConstPtr getMessage() const override;

    virtual void clearBuffer() override;

private:
    std::vector<DynamicInput*> correspondents_;

    std::deque<Token::ConstPtr> messages_to_send_;
    std::deque<Token::ConstPtr> committed_messages_;
    TokenConstPtr current_message_;
};
}

#endif // DYNAMIC_OUTPUT_H

