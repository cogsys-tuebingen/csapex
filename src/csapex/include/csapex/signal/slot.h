#ifndef SLOT_H
#define SLOT_H

/// COMPONENT
#include <csapex/model/connectable.h>
#include <csapex/csapex_fwd.h>

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
    Slot(std::function<void()> callback, Unique *parent, int sub_id, bool active);
    virtual ~Slot();

    virtual void trigger(Trigger *source);

    virtual bool canInput() const {
        return true;
    }
    virtual bool isInput() const {
        return true;
    }

    bool isActive() const;

    virtual bool canConnectTo(Connectable* other_side, bool move) const;


    virtual bool targetsCanBeMovedTo(Connectable* other_side) const;
    virtual bool isConnected() const;

    virtual void connectionMovePreview(Connectable* other_side);
    virtual void validateConnections();

    std::vector<Trigger*> getSources() const;

    virtual void enable();
    virtual void disable();

    virtual void notifyMessageProcessed();

    void reset();

    void handleTrigger();

public:
    boost::signals2::signal<void(Trigger*)> triggered;

protected:
    virtual bool isConnectionPossible(Connectable* other_side);
    virtual bool acknowledgeConnection(Connectable* other_side);
    virtual void removeConnection(Connectable* other_side);
    virtual void removeAllConnectionsNotUndoable();

protected:
    std::vector<Trigger*> sources_;

    std::function<void()> callback_;
    bool active_;
};

}
#endif // SLOT_H
