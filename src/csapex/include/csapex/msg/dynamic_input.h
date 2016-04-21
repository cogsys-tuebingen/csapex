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
    bool inputMessagePart(const TokenConstPtr& msg);
    std::vector<TokenConstPtr> getMessageParts() const;
    void composeMessage();

    virtual bool hasMessage() const override;
    virtual bool hasReceived() const override;

    virtual TokenConstPtr getMessage() const override;

private:
    DynamicOutput* correspondent_;

    std::vector<TokenConstPtr> msg_parts_;
    std::vector<TokenConstPtr> composed_msg_;
};

}

#endif // DYNAMIC_INPUT_H

