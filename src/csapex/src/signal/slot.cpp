/// HEADER
#include <csapex/signal/slot.h>

/// COMPONENT
#include <csapex/signal/trigger.h>
#include <csapex/signal/signal.h>
#include <csapex/command/delete_connection.h>
#include <csapex/command/command.h>
#include <csapex/utility/assert.h>
#include <csapex/command/meta.h>

/// SYSTEM
#include <iostream>
#include <QFuture>

using namespace csapex;

Slot::Slot(std::function<void()> callback, const UUID &uuid, bool active)
    : Connectable(uuid), callback_(callback), active_(active)
{
    setType(connection_types::makeEmpty<connection_types::Signal>());
//    QObject::connect(this, SIGNAL(triggered()), this, SLOT(handleTrigger()), Qt::QueuedConnection);
}

Slot::Slot(std::function<void()> callback, Unique* parent, int sub_id, bool active)
    : Connectable(parent, sub_id, "slot"), callback_(callback), active_(active)
{
    setType(connection_types::makeEmpty<connection_types::Signal>());
//    QObject::connect(this, SIGNAL(triggered()), this, SLOT(handleTrigger()), Qt::QueuedConnection);
}

Slot::~Slot()
{
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
    Trigger* target = dynamic_cast<Trigger*>(other_side);

    sources_.push_back(target);

    connect(other_side, SIGNAL(destroyed(QObject*)), this, SLOT(removeConnection(QObject*)), Qt::DirectConnection);
    connect(other_side, SIGNAL(enabled(bool)), this, SIGNAL(connectionEnabled(bool)));
    return true;
}

void Slot::removeConnection(Connectable* other_side)
{
    std::vector<Trigger*>::iterator pos = std::find(sources_.begin(), sources_.end(), other_side);
    if(pos != sources_.end()) {
        sources_.erase(pos);

        Q_EMIT connectionRemoved(this);
    }
}

Command::Ptr Slot::removeAllConnectionsCmd()
{
    command::Meta::Ptr cmd(new command::Meta("Delete sources"));
    foreach(Trigger* source, sources_) {
        cmd->add(Command::Ptr(new command::DeleteConnection(source, this)));
    }
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
    for(std::vector<Trigger*>::iterator i = sources_.begin(); i != sources_.end();) {
        Connectable* target = *i;
        target->removeConnection(this);
        i = sources_.erase(i);
    }

    Q_EMIT disconnected(this);
}


bool Slot::canConnectTo(Connectable* other_side, bool /*move*/) const
{
    Trigger* trigger = dynamic_cast<Trigger*>(other_side);
    return trigger;
}

bool Slot::targetsCanBeMovedTo(Connectable* other_side) const
{
    foreach(Trigger* trigger, sources_) {
        if(!trigger->canConnectTo(other_side, true)/* || !canConnectTo(*it)*/) {
            return false;
        }
    }
    return true;
}

bool Slot::isConnected() const
{
    return !sources_.empty();
}

void Slot::connectionMovePreview(Connectable *other_side)
{
    foreach(Trigger* trigger, sources_) {
        Q_EMIT(connectionInProgress(trigger, other_side));
    }
}


void Slot::validateConnections()
{
}

std::vector<Trigger*> Slot::getSources() const
{
    return sources_;
}

void Slot::trigger()
{
    std::unique_lock<std::mutex> lock(trigger_exec_mutex_);

    Q_EMIT triggered();

    // wait for the signal to be handled
    exec_finished_.wait(lock);
}

void Slot::handleTrigger()
{
    std::unique_lock<std::mutex> lock(trigger_exec_mutex_);

    // do the work
    if(isEnabled() || isActive()) {
        callback_();
    }

    lock.unlock();

    exec_finished_.notify_all();
}

void Slot::notifyMessageProcessed()
{
    Connectable::notifyMessageProcessed();

    foreach(Trigger* trigger, sources_) {
        trigger->notifyMessageProcessed();
    }
}

bool Slot::isActive() const
{
    return active_;
}
