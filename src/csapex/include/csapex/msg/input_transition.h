#ifndef INPUT_TRANSITION_H
#define INPUT_TRANSITION_H

/// COMPONENT
#include <csapex/msg/transition.h>

namespace csapex
{

class InputTransition : public Transition
{
public:
    InputTransition(NodeWorker* node);

    void notifyMessageProcessed();
    void update();

protected:
    virtual void connectionAdded(Connection* connection) override;

private:
    void fire();

    bool areConnectionsReady() const;

private:
    bool one_input_is_dynamic_;
};

}

#endif // INPUT_TRANSITION_H

