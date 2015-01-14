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
    Q_OBJECT

    friend class Trigger;
    friend class command::AddConnection;
    friend class command::MoveConnection;
    friend class command::DeleteConnection;

public:
    Slot(std::function<void()> callback, const UUID &uuid, bool active);
    Slot(std::function<void()> callback, Unique *parent, int sub_id, bool active);
    virtual ~Slot();

    virtual void trigger();

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

    virtual CommandPtr removeAllConnectionsCmd();

    virtual void enable();
    virtual void disable();

    virtual void notifyMessageProcessed();

    void reset();

protected:
    virtual bool tryConnect(Connectable* other_side);
    virtual bool acknowledgeConnection(Connectable* other_side);
    virtual void removeConnection(Connectable* other_side);
    virtual void removeAllConnectionsNotUndoable();

Q_SIGNALS:
    void triggered();

private Q_SLOTS:
    void handleTrigger();

protected:
    std::vector<Trigger*> sources_;

    std::mutex trigger_exec_mutex_;
    std::condition_variable exec_finished_;

    std::function<void()> callback_;
    bool active_;
};

}
#endif // SLOT_H
