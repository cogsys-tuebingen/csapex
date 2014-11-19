/// HEADER
#include <csapex/signal/slot.h>

/// COMPONENT
#include <csapex/signal/trigger.h>
#include <csapex/signal/signal.h>
#include <csapex/command/delete_connection.h>
#include <csapex/command/command.h>
#include <csapex/utility/assert.h>

/// SYSTEM
#include <iostream>
#include <QFuture>

using namespace csapex;

Slot::Slot(boost::function<void()> callback, Settings& settings, const UUID &uuid)
    : Connectable(settings, uuid), target(NULL), callback_(callback)
{
    setType(connection_types::makeEmpty<connection_types::Signal>());
    QObject::connect(this, SIGNAL(triggered()), this, SLOT(handleTrigger()), Qt::QueuedConnection);
}

Slot::Slot(boost::function<void()> callback, Settings &settings, Unique* parent, int sub_id)
    : Connectable(settings, parent, sub_id, "slot"), target(NULL), callback_(callback)
{
    setType(connection_types::makeEmpty<connection_types::Signal>());
    QObject::connect(this, SIGNAL(triggered()), this, SLOT(handleTrigger()), Qt::QueuedConnection);
}

Slot::~Slot()
{
    if(target != NULL) {
        target->removeConnection(this);
    }
}

void Slot::reset()
{
    setSequenceNumber(0);
}

bool Slot::tryConnect(Connectable* other_side)
{
    if(!other_side->canOutput()) {
        std::cerr << "cannot connect, other side can't output" << std::endl;
        return false;
    }

    return other_side->tryConnect(this);
}

bool Slot::acknowledgeConnection(Connectable* other_side)
{
    target = dynamic_cast<Trigger*>(other_side);
    connect(other_side, SIGNAL(destroyed(QObject*)), this, SLOT(removeConnection(QObject*)), Qt::DirectConnection);
    connect(other_side, SIGNAL(enabled(bool)), this, SIGNAL(connectionEnabled(bool)));
    return true;
}

void Slot::removeConnection(Connectable* other_side)
{
    if(target != NULL) {
        apex_assert_hard(other_side == target);
        target = NULL;

        Q_EMIT connectionRemoved(this);
    }
}

Command::Ptr Slot::removeAllConnectionsCmd()
{
    Command::Ptr cmd(new command::DeleteConnection(target, this));
    return cmd;
}


void Slot::enable()
{
    Connectable::enable();
    //    if(isConnected() && !getSource()->isEnabled()) {
    //        getSource()->enable();
    //    }
}

void Slot::disable()
{
    Connectable::disable();

    //    if(isBlocked()) {
    notifyMessageProcessed();
    //    }
    //    if(isConnected() && getSource()->isEnabled()) {
    //        getSource()->disable();
    //    }
}

void Slot::removeAllConnectionsNotUndoable()
{
    if(target != NULL) {
        target->removeConnection(this);
        target = NULL;
        setError(false);
        Q_EMIT disconnected(this);
    }
}


bool Slot::canConnectTo(Connectable* other_side, bool move) const
{
    Trigger* trigger = dynamic_cast<Trigger*>(other_side);
    return trigger;
}

bool Slot::targetsCanBeMovedTo(Connectable* other_side) const
{
    return target->canConnectTo(other_side, true) /*&& canConnectTo(getConnected())*/;
}

bool Slot::isConnected() const
{
    return target != NULL;
}

void Slot::connectionMovePreview(Connectable *other_side)
{
    Q_EMIT(connectionInProgress(getSource(), other_side));
}


void Slot::validateConnections()
{
    bool e = false;
    if(isConnected()) {
        ConnectionType::ConstPtr target_type = target->getType();
        if(!target_type) {
            e = true;
        } else if(!target_type->canConnectTo(getType().get())) {
            e = true;
        }
    }

    setError(e);
}

Connectable *Slot::getSource() const
{
    return target;
}

void Slot::trigger()
{
    trigger_exec_mutex_.lock();

    Q_EMIT triggered();

    // wait for the signal to be handled
    exec_finished_.wait(&trigger_exec_mutex_);
    trigger_exec_mutex_.unlock();
}

void Slot::handleTrigger()
{
    QMutexLocker lock(&trigger_exec_mutex_);

    // do the work
    callback_();

    exec_finished_.wakeAll();
}

void Slot::notifyMessageProcessed()
{
    Connectable::notifyMessageProcessed();

    if(target) {
        target->notifyMessageProcessed();
    }
}
