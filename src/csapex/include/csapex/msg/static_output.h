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

    virtual void addMessage(Token::ConstPtr message) override;

    virtual void setMultipart(bool multipart, bool last_part) override;

    virtual void commitMessages(bool is_activated) override;
    virtual bool hasMessage() override;
    virtual bool hasMarkerMessage() override;
    virtual void nextMessage() override;
    virtual TokenConstPtr getMessage() const override;

    virtual void disable() override;
    virtual void reset() override;
    virtual void clearBuffer() override;

    Token::ConstPtr getMessage();

private:
    Token::ConstPtr message_to_send_;
    Token::ConstPtr committed_message_;

    int message_flags_;
};
}

#endif // STATIC_OUTPUT_H

