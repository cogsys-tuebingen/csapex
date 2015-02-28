#ifndef OUTPUT_TRANSITION_H
#define OUTPUT_TRANSITION_H

/// COMPONENT
#include <csapex/msg/transition.h>

namespace csapex
{
class OutputTransition : public Transition
{
public:
    OutputTransition(NodeWorker* node);

    bool isSink() const;
    bool canSendMessages() const;
    void sendMessages();
    void notifyMessageProcessed();

    void clearOutputs();
    void setConnectionsReadyToReceive();
    bool areOutputsIdle() const;

private:

    void fillConnections();

private:
    bool notify_called_;
};
}

#endif // OUTPUT_TRANSITION_H

