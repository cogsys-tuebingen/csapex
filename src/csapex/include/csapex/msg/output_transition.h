#ifndef OUTPUT_TRANSITION_H
#define OUTPUT_TRANSITION_H

/// COMPONENT
#include <csapex/msg/transition.h>
#include <csapex/utility/uuid.h>

/// SYSTEM
#include <unordered_map>

namespace csapex
{
class OutputTransition : public Transition
{
public:
    OutputTransition();

    void addOutput(OutputPtr output);
    void removeOutput(OutputPtr output);

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

public:
    boost::signals2::signal<void()> messages_processed;

private:
    void fillConnections();

private:
    std::unordered_map<OutputPtr, std::vector<boost::signals2::connection>> output_signal_connections_;

    std::unordered_map<UUID, OutputPtr, UUID::Hasher> outputs_;
};
}

#endif // OUTPUT_TRANSITION_H

