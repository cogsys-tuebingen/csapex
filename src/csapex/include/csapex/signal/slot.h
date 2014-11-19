#ifndef SLOT_H
#define SLOT_H

/// COMPONENT
#include <csapex/model/connectable.h>
#include <csapex/csapex_fwd.h>
#include <csapex/msg/message.h>
#include <csapex/msg/generic_pointer_message.hpp>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/utility/buffer.h>

/// SYSTEM
#include <QMutex>
#include <QWaitCondition>
#include <QFuture>

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
    Slot(Settings &settings, const UUID &uuid);
    Slot(Settings& settings, Unique *parent, int sub_id);
    virtual ~Slot();

    virtual void trigger();

    virtual bool canInput() const {
        return true;
    }
    virtual bool isInput() const {
        return true;
    }

    virtual bool canConnectTo(Connectable* other_side, bool move) const;


    virtual bool targetsCanBeMovedTo(Connectable* other_side) const;
    virtual bool isConnected() const;

    virtual void connectionMovePreview(Connectable* other_side);
    virtual void validateConnections();

    Connectable* getSource() const;

    virtual CommandPtr removeAllConnectionsCmd();

    bool isOptional() const;
    void setOptional(bool optional);

    template <typename T>
    bool isMessage() {
        return buffer_->isType<T>();
    }

    bool hasMessage() const;
    bool hasReceived() const;

    void free();
    void stop();

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
    Connectable* target;

    QMutex trigger_exec_mutex_;
    QWaitCondition exec_finished_;

    BufferPtr buffer_;

    bool optional_;
};

}
#endif // SLOT_H
