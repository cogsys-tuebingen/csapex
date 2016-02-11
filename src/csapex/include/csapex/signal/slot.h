#ifndef SLOT_H
#define SLOT_H

/// COMPONENT
#include <csapex/signal/signal_fwd.h>
#include <csapex/model/connectable.h>

/// SYSTEM
#include <mutex>
#include <condition_variable>

namespace csapex
{

class Slot : public Connectable
{
    friend class Trigger;

public:
    Slot(std::function<void()> callback, const UUID &uuid, bool active);
    virtual ~Slot();

    virtual void trigger(Trigger *source);

    virtual bool canInput() const override {
        return true;
    }
    virtual bool isInput() const override {
        return true;
    }

    bool isActive() const;

    virtual bool canConnectTo(Connectable* other_side, bool move) const override;


    virtual bool targetsCanBeMovedTo(Connectable* other_side) const override;
    virtual bool isConnected() const override;

    virtual void connectionMovePreview(Connectable* other_side) override;
    virtual void validateConnections() override;

    std::vector<Trigger*> getSources() const;

    virtual void enable() override;
    virtual void disable() override;

    virtual void notifyMessageProcessed() override;

    void reset();

    void handleTrigger();

public:
    csapex::slim_signal::Signal<void(Trigger*)> triggered;
    csapex::slim_signal::Signal<void()> connected;

protected:
    bool acknowledgeConnection(Connectable* other_side);

    virtual bool isConnectionPossible(Connectable* other_side) override;
    virtual void removeConnection(Connectable* other_side) override;
    virtual void removeAllConnectionsNotUndoable() override;

protected:
    std::vector<Trigger*> sources_;

    std::function<void()> callback_;
    bool active_;
};

}
#endif // SLOT_H
