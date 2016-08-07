#ifndef DYNAMIC_INPUT_H
#define DYNAMIC_INPUT_H

/// COMPONENT
#include <csapex/msg/input.h>

namespace csapex
{
class DynamicInput : public Input
{
public:
    DynamicInput(const UUID &uuid);

    virtual bool hasMessage() const override;
    virtual bool hasReceived() const override;

    virtual void setToken(TokenPtr message) override;
    virtual TokenPtr getToken() const override;

    std::vector<TokenPtr> getMessageParts() const;

    virtual void notifyMessageProcessed() override;

private:
    void composeMessage();

private:
    std::vector<TokenPtr> msg_parts_;
    std::vector<TokenPtr> composed_msg_;

    bool has_last_;
};

}

#endif // DYNAMIC_INPUT_H

