#ifndef TRIGGER_H
#define TRIGGER_H

/// COMPONENT
#include <csapex/signal/signal_fwd.h>
#include <csapex/model/connectable.h>
#include <csapex/msg/generic_pointer_message.hpp>
#include <csapex/msg/generic_value_message.hpp>

namespace csapex
{

class Trigger : public Connectable
{
    friend class Input;
    friend class Graph;

public:
    Trigger(const UUID &uuid);
    ~Trigger();

    virtual bool canOutput() const override {
        return true;
    }
    virtual bool isOutput() const override {
        return true;
    }
    virtual ConnectorType getConnectorType() const override
    {
        return ConnectorType::TRIGGER;
    }

    void trigger();
    void signalHandled(Slot* slot);

    virtual void disable() override;

    virtual bool canConnectTo(Connectable* other_side, bool move) const override;

    virtual bool targetsCanBeMovedTo(Connectable *other_side) const override;
    virtual bool isConnected() const override;

    virtual void connectionMovePreview(Connectable* other_side) override;
    virtual void validateConnections() override;

    int noTargets();
    std::vector<Slot*> getTargets() const;

    void reset();

    bool isBeingProcessed() const;

    virtual bool isConnectionPossible(Connectable* other_side) override;
    virtual void removeConnection(Connectable* other_side) override;
    virtual void removeAllConnectionsNotUndoable() override;

    bool connect(Slot* other_side);

public:
    csapex::slim_signal::Signal<void()> triggered;
    csapex::slim_signal::Signal<void()> all_signals_handled;

protected:
    std::vector<Slot*> targets_;

    mutable std::recursive_mutex targets_running_mtx_;
    std::map<Slot*, bool> targets_running_;

};

}

#endif // TRIGGER_H
