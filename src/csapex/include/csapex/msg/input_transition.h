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
    void fireIfPossible();

    void establishConnections() override;
    void connectionRemoved(Connection* connection) override;

    virtual void reset() override;

protected:
    virtual void connectionAdded(Connection* connection) override;

private:
    void fire();
    int findHighestDeviantSequenceNumber() const;
    void notifyOlderConnections(int highest_seq);

    bool areConnectionsReady() const;

private:
    bool one_input_is_dynamic_;
};

}

#endif // INPUT_TRANSITION_H

