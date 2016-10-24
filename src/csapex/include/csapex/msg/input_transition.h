#ifndef INPUT_TRANSITION_H
#define INPUT_TRANSITION_H

/// COMPONENT
#include <csapex/msg/transition.h>
#include <csapex/utility/uuid.h>
#include <csapex/model/node_runner.h>

/// SYSTEM
#include <unordered_map>

namespace csapex
{

class CSAPEX_EXPORT InputTransition : public Transition
{
public:
    InputTransition(delegate::Delegate0<> activation_fn);
    InputTransition();

    InputPtr getInput(const UUID& id) const;
    std::vector<UUID> getInputs() const;

    void addInput(InputPtr input);
    void removeInput(InputPtr input);

    void notifyMessageRead();
    void notifyMessageProcessed();

    void forwardMessages();
    bool areMessagesForwarded() const;
    bool areMessagesProcessed() const;
    bool areMessagesComplete() const;

    int findHighestDeviantSequenceNumber() const;

    virtual bool isEnabled() const override;

    virtual void connectionRemoved(Connection* connection) override;

    virtual void reset() override;

protected:
    virtual void connectionAdded(Connection* connection) override;

private:

    bool areConnectionsReady() const;

private:
    std::map<InputPtr, std::vector<csapex::slim_signal::Connection>> input_signal_connections_;

    std::unordered_map<UUID, InputPtr, UUID::Hasher> inputs_;

    bool forwarded_;
    bool processed_;
};

}

#endif // INPUT_TRANSITION_H

