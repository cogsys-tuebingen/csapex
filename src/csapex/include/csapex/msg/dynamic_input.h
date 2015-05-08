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
    DynamicInput(InputTransition* transition, const UUID &uuid);
    DynamicInput(InputTransition* transition, Unique *parent, int sub_id);

    void setCorrespondent(DynamicOutput* output);

    /**
     * @brief inputMessagePart
     * @param msg
     * @return true, iff every message part has been received
     */
    bool inputMessagePart(const ConnectionTypeConstPtr& msg);
    std::vector<ConnectionTypeConstPtr> getMessageParts() const;
    void composeMessage();

    virtual bool hasMessage() const override;
    virtual bool hasReceived() const override;

    virtual ConnectionTypeConstPtr getMessage() const override;

private:
    DynamicOutput* correspondent_;

    std::vector<ConnectionTypeConstPtr> msg_parts_;
    std::vector<ConnectionTypeConstPtr> composed_msg_;
};

}

#endif // DYNAMIC_INPUT_H

