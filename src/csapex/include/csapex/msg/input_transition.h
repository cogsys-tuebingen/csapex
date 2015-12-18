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

class InputTransition : public Transition
{
public:
    InputTransition(delegate::Delegate0<> activation_fn);
    InputTransition();

    void addInput(InputPtr input);
    void removeInput(InputPtr input);

    void notifyMessageProcessed();

    void forwardMessages();

    int findHighestDeviantSequenceNumber() const;
    void notifyOlderConnections(int highest_seq);

    virtual bool isEnabled() const override;

    virtual void establishConnections() override;
    virtual void connectionRemoved(Connection* connection) override;

    virtual void reset() override;

protected:
    virtual void connectionAdded(Connection* connection) override;

private:

    bool areConnectionsReady() const;

private:
    bool one_input_is_dynamic_;

    std::map<InputPtr, std::vector<boost::signals2::connection>> input_signal_connections_;

    std::unordered_map<UUID, InputPtr, UUID::Hasher> inputs_;
};

}

#endif // INPUT_TRANSITION_H

