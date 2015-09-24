#ifndef INPUT_TRANSITION_H
#define INPUT_TRANSITION_H

/// COMPONENT
#include <csapex/msg/transition.h>
#include <csapex/utility/uuid.h>

/// SYSTEM
#include <unordered_map>

namespace csapex
{

class InputTransition : public Transition
{
public:
    InputTransition(NodeWorker* node);

    void addInput(InputPtr input);
    void removeInput(InputPtr input);

    void notifyMessageProcessed();
    void fireIfPossible();

    bool isEnabled() const;

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
    NodeWorker* node_;

    bool one_input_is_dynamic_;

    std::map<InputPtr, std::vector<boost::signals2::connection>> input_signal_connections_;

    std::unordered_map<UUID, InputPtr, UUID::Hasher> inputs_;
};

}

#endif // INPUT_TRANSITION_H

