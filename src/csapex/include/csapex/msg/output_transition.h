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

    void addOutput(OutputPtr output);
    void removeOutput(OutputPtr output);

    void connectionAdded(Connection* connection);
    void connectionRemoved(Connection *connection);

    bool isSink() const;
    bool canStartSendingMessages() const;
    void sendMessages();
    void publishNextMessage();

    void clearOutputs();
    void abortSendingMessages();
    void setConnectionsReadyToReceive();
    bool areOutputsIdle() const;

    virtual void reset() override;

    void establishConnections();

private:
    void fillConnections();

private:
    std::map<OutputPtr, std::vector<boost::signals2::connection>> output_signal_connections_;
};
}

#endif // OUTPUT_TRANSITION_H

