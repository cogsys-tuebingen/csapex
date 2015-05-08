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

    void connectionAdded(Connection* connection);

    bool isSink() const;
    bool canSendMessages() const;
    void sendMessages();
    void updateOutputs();

    void clearOutputs();
    void setConnectionsReadyToReceive();
    bool areOutputsIdle() const;

    void establish();

private:

    void fillConnections();

private:
    bool outputs_done_;
};
}

#endif // OUTPUT_TRANSITION_H

