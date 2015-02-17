/// HEADER
#include <csapex/signal/trigger.h>

/// COMPONENT
#include <csapex/signal/slot.h>
#include <csapex/signal/signal.h>
#include <csapex/command/meta.h>
#include <csapex/command/delete_connection.h>
#include <csapex/utility/timer.h>
#include <csapex/utility/assert.h>
#include <csapex/msg/message_traits.h>

/// SYSTEM

#include <iostream>

using namespace csapex;

Trigger::Trigger(const UUID& uuid)
    : Connectable(uuid)
{
    setType(connection_types::makeEmpty<connection_types::Signal>());
}

Trigger::Trigger(Unique* parent, int sub_id)
    : Connectable(parent, sub_id, "trigger")
{
    setType(connection_types::makeEmpty<connection_types::Signal>());
}

Trigger::~Trigger()
{
}

void Trigger::reset()
{
    setBlocked(false);
    setSequenceNumber(0);
}

int Trigger::noTargets()
{
    return targets_.size();
}

std::vector<Slot*> Trigger::getTargets() const
{
    return targets_;
}

void Trigger::removeConnection(Connectable* other_side)
{
    for(std::vector<Slot*>::iterator i = targets_.begin(); i != targets_.end();) {
        if(*i == other_side) {
            other_side->removeConnection(this);

            i = targets_.erase(i);

            Q_EMIT connectionRemoved(this);
            return;

        } else {
            ++i;
        }
    }
}

Command::Ptr Trigger::removeConnectionCmd(Slot* other_side) {
    Command::Ptr removeThis(new command::DeleteConnection(this, other_side));

    return removeThis;
}

Command::Ptr Trigger::removeAllConnectionsCmd()
{
    command::Meta::Ptr removeAll(new command::Meta("Remove All Connections"));

    foreach(Slot* target, targets_) {
        Command::Ptr removeThis(new command::DeleteConnection(this, target));
        removeAll->add(removeThis);
    }

    return removeAll;
}

void Trigger::removeAllConnectionsNotUndoable()
{
    for(std::vector<Slot*>::iterator i = targets_.begin(); i != targets_.end();) {
        (*i)->removeConnection(this);
        i = targets_.erase(i);
    }

    Q_EMIT disconnected(this);
}

void Trigger::trigger()
{
    foreach(Slot* s, targets_) {
        try {
            s->trigger();
        } catch(const std::exception& e) {
            std::cout << "triggering slot " << s->getLabel()  << " failed: " << e.what();
        }
    }
    ++count_;
}

void Trigger::disable()
{
    Connectable::disable();
}

bool Trigger::isConnectionPossible(Connectable *other_side)
{
    return connect(other_side);
}

bool Trigger::connect(Connectable *other_side)
{
    Slot* slot = dynamic_cast<Slot*>(other_side);
    if(!slot) {
        return false;
    }

    apex_assert_hard(slot);

    if(!slot->acknowledgeConnection(this)) {
        std::cerr << "cannot connect, slot doesn't acknowledge" << std::endl;
        return false;
    }

    targets_.push_back(slot);

    QObject::connect(other_side, SIGNAL(destroyed(QObject*)), this, SLOT(removeConnection(QObject*)), Qt::DirectConnection);

    validateConnections();

    return true;
}

bool Trigger::canConnectTo(Connectable* other_side, bool move) const
{
    if(!Connectable::canConnectTo(other_side, move)) {
        return false;
    } else {
        foreach (Slot* target, targets_) {
            if(target == other_side) {
                return false;
            }
        }
    }
    return true;
}


bool Trigger::targetsCanBeMovedTo(Connectable* other_side) const
{
    foreach(Slot* slot, targets_) {
        if(!slot->canConnectTo(other_side, true)/* || !canConnectTo(*it)*/) {
            return false;
        }
    }
    return true;
}

bool Trigger::isConnected() const
{
    return targets_.size() > 0;
}

void Trigger::connectionMovePreview(Connectable *other_side)
{
    foreach(Slot* slot, targets_) {
        Q_EMIT(connectionInProgress(slot, other_side));
    }
}

void Trigger::validateConnections()
{
    foreach(Slot* target, targets_) {
        target->validateConnections();
    }
}
