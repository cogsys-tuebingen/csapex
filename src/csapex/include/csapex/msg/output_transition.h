#ifndef OUTPUT_TRANSITION_H
#define OUTPUT_TRANSITION_H

/// COMPONENT
#include <csapex/msg/transition.h>
#include <csapex/utility/uuid.h>

/// SYSTEM
#include <unordered_map>

namespace csapex
{
class CSAPEX_EXPORT OutputTransition : public Transition
{
public:
    OutputTransition(delegate::Delegate0<> activation_fn);
    OutputTransition();

    std::vector<UUID> getOutputs() const;
    OutputPtr getOutput(const UUID& id) const;

    void addOutput(OutputPtr output);
    void removeOutput(OutputPtr output);

    void setSequenceNumber(long seq_no);
    long getSequenceNumber() const;

    void connectionAdded(Connection* connection) override;
    void connectionRemoved(Connection *connection) override;

    bool canStartSendingMessages() const;
    bool sendMessages(bool is_active);
    void publishNextMessage();

    void clearBuffer();
    void setOutputsIdle();
    bool areOutputsIdle() const;

    virtual bool isEnabled() const override;

    virtual void reset() override;

public:
    slim_signal::Signal<void()> messages_processed;

private:
    void fillConnections();

private:
    std::unordered_map<OutputPtr, std::vector<slim_signal::Connection>> output_signal_connections_;
    std::unordered_map<UUID, OutputPtr, UUID::Hasher> outputs_;

    long sequence_number_;
    bool try_to_publish_;
};
}

#endif // OUTPUT_TRANSITION_H

