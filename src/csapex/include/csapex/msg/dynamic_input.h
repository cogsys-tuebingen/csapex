#ifndef DYNAMIC_INPUT_H
#define DYNAMIC_INPUT_H

/// COMPONENT
#include <csapex/msg/input.h>

namespace csapex
{
class DynamicOutput;

class DynamicInput : public Input
{
public:
    DynamicInput(const UUID &uuid);

    void setCorrespondent(DynamicOutput* output);

    /**
     * @brief inputMessagePart
     * @param msg
     * @return true, iff every message part has been received
     */
    bool inputMessagePart(const TokenPtr& msg);
    std::vector<TokenPtr> getMessageParts() const;
    void composeMessage();

    virtual bool hasMessage() const override;
    virtual bool hasReceived() const override;

    virtual TokenPtr getToken() const override;

private:
    DynamicOutput* correspondent_;

    std::vector<TokenPtr> msg_parts_;
    std::vector<TokenPtr> composed_msg_;
};

}

#endif // DYNAMIC_INPUT_H

