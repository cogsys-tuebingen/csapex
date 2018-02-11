#ifndef OUTPUT_TRANSITION_H
#define OUTPUT_TRANSITION_H

/// COMPONENT
#include <csapex/msg/transition.h>
#include <csapex/utility/uuid.h>

/// SYSTEM
#include <unordered_map>

namespace csapex
{
class CSAPEX_CORE_EXPORT OutputTransition : public Transition
{
public:
    OutputTransition(delegate::Delegate0<> activation_fn);
    OutputTransition();

    int getPortCount() const override;

    std::vector<UUID> getOutputs() const;
    OutputPtr getOutput(const UUID& id) const;
    OutputPtr getOutputNoThrow(const UUID& id) const noexcept;

    void addOutput(OutputPtr output);
    void removeOutput(OutputPtr output);

    void setSequenceNumber(long seq_no);
    long getSequenceNumber() const;

    bool canStartSendingMessages() const;
    bool sendMessages(bool is_active);
    void tokenProcessed();

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
    std::unordered_map<Output*, std::vector<slim_signal::ScopedConnection>> output_signal_connections_;
    std::unordered_map<UUID, OutputPtr, UUID::Hasher> outputs_;

    long sequence_number_;
};
}

#endif // OUTPUT_TRANSITION_H

