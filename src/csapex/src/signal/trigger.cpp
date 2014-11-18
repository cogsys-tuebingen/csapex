/// HEADER
#include <csapex/signal/trigger.h>

/// COMPONENT
#include <csapex/signal/slot.h>
#include <csapex/command/meta.h>
#include <csapex/command/delete_connection.h>
#include <csapex/utility/timer.h>
#include <csapex/utility/assert.h>
#include <csapex/msg/message_traits.h>

/// SYSTEM
#include <boost/foreach.hpp>
#include <iostream>

using namespace csapex;

Trigger::Trigger(Settings &settings, const UUID& uuid)
    : Connectable(settings, uuid), force_send_message_(false)
{
}

Trigger::Trigger(Settings& settings, Unique* parent, int sub_id)
    : Connectable(settings, parent, sub_id, "trigger"), force_send_message_(false)
{
}

Trigger::~Trigger()
{
    foreach(Slot* i, targets_) {
        i->removeConnection(this);
    }
}

void Trigger::reset()
{
    setBlocked(false);
    setSequenceNumber(0);
    message_.reset();
    message_to_send_.reset();
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

void Trigger::disable()
{
    Connectable::disable();

//    if(isBlocked()) {
        message_to_send_.reset();
        message_.reset();
//    }
}

void Trigger::connectForcedWithoutCommand(Slot *other_side)
{
    tryConnect(other_side);
}

bool Trigger::tryConnect(Connectable *other_side)
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
    targets_.push_back(slot);

    QObject::connect(other_side, SIGNAL(destroyed(QObject*)), this, SLOT(removeConnection(QObject*)), Qt::DirectConnection);

    validateConnections();

    return true;
}

bool Trigger::targetsCanBeMovedTo(Connectable* other_side) const
{
    foreach(Slot* Slot, targets_) {
        if(!Slot->canConnectTo(other_side, true)/* || !canConnectTo(*it)*/) {
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
    foreach(Slot* Slot, targets_) {
        Q_EMIT(connectionInProgress(Slot, other_side));
    }
}

void Trigger::validateConnections()
{
    foreach(Slot* target, targets_) {
        target->validateConnections();
    }
}

void Trigger::publish(ConnectionType::Ptr message)
{
    setType(message);

    // update buffer
    message_to_send_ = message;

    setBlocked(true);
}

bool Trigger::hasMessage()
{
    return message_to_send_;
}

bool Trigger::canSendMessages()
{
    foreach(Slot* Slot, targets_) {
        bool blocked = Slot->isEnabled() && Slot->isBlocked();
        if(blocked) {
            return false;
        }
    }
    return true;
}

void Trigger::sendMessages()
{
    if(message_to_send_) {
        message_ = message_to_send_;
        message_to_send_.reset();

    } else {
        if(!targets_.empty()) {
//            std::cout << getUUID() << " sends empty message" << std::endl;
        }
        message_ = connection_types::makeEmpty<connection_types::NoMessage>();
    }

    message_->setSequenceNumber(seq_no_);

    // wait for all connected Slots to be able to receive
    //  * Slots can only be connected to this Trigger since they are 1:1
    std::vector<Slot*> targets;
    foreach(Slot* i, targets_) {
        if(i->isEnabled()) {
            targets.push_back(i);
        }
    }

    if(!targets.empty()) {
        // all connected Slots are ready to receive, send them the message
        if(targets.size() == 1) {
            targets[0]->inputMessage(message_);
        } else if(targets.size() > 1) {
            foreach(Slot* i, targets) {
                ConnectionType::Ptr msg = message_->clone();
                msg->setSequenceNumber(seq_no_);
                i->inputMessage(msg);
            }
        }
        ++count_;
    }

    setBlocked(false);

    ++seq_no_;
    Q_EMIT messageSent(this);
}

void Trigger::forceSendMessage(bool force)
{
    force_send_message_ = force;
}

ConnectionType::Ptr Trigger::getMessage()
{
    return message_;
}
