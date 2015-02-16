/// HEADER
#include <csapex/msg/output.h>

/// COMPONENT
#include <csapex/msg/input.h>
#include <csapex/command/meta.h>
#include <csapex/command/delete_connection.h>
#include <csapex/utility/timer.h>
#include <csapex/utility/assert.h>
#include <csapex/msg/message_traits.h>

/// SYSTEM
#include <iostream>

using namespace csapex;

Output::Output(const UUID& uuid)
    : Connectable(uuid), force_send_message_(false)
{
}

Output::Output(Unique* parent, int sub_id)
    : Connectable(parent, sub_id, "out"), force_send_message_(false)
{
}

Output::~Output()
{
}

void Output::reset()
{
    clear();

    setBlocked(false);
    setSequenceNumber(0);
}

int Output::noTargets()
{
    return targets_.size();
}

std::vector<Input*> Output::getTargets() const
{
    return targets_;
}

void Output::removeConnection(Connectable* other_side)
{
    for(std::vector<Input*>::iterator i = targets_.begin(); i != targets_.end();) {
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

Command::Ptr Output::removeConnectionCmd(Input* other_side) {
    Command::Ptr removeThis(new command::DeleteConnection(this, other_side));

    return removeThis;
}

Command::Ptr Output::removeAllConnectionsCmd()
{
    command::Meta::Ptr removeAll(new command::Meta("Remove All Connections"));

    foreach(Input* target, targets_) {
        Command::Ptr removeThis(new command::DeleteConnection(this, target));
        removeAll->add(removeThis);
    }

    return removeAll;
}

void Output::removeAllConnectionsNotUndoable()
{
    for(std::vector<Input*>::iterator i = targets_.begin(); i != targets_.end();) {
        (*i)->removeConnection(this);
        i = targets_.erase(i);
    }

    Q_EMIT disconnected(this);
}

void Output::disable()
{
    Connectable::disable();
}

void Output::connectForcedWithoutCommand(Input *other_side)
{
    tryConnect(other_side);
}

bool Output::tryConnect(Connectable *other_side)
{
    return connect(other_side);
}

bool Output::connect(Connectable *other_side)
{
    if(!other_side->canInput()) {
        std::cerr << "cannot connect, other side can't input" << std::endl;
        return false;
    }
    if(!other_side->canConnectTo(this, false)) {
        std::cerr << "cannot connect, other side can't connect" << std::endl;
        return false;
    }

    Input* input = dynamic_cast<Input*>(other_side);

    if(!input->acknowledgeConnection(this)) {
        std::cerr << "cannot connect, other side doesn't acknowledge" << std::endl;
        return false;
    }

    apex_assert_hard(input);
    targets_.push_back(input);

    QObject::connect(other_side, SIGNAL(destroyed(QObject*)), this, SLOT(removeConnection(QObject*)), Qt::DirectConnection);

    validateConnections();

    return true;
}

bool Output::targetsCanBeMovedTo(Connectable* other_side) const
{
    foreach(Input* input, targets_) {
        if(!input->canConnectTo(other_side, true)/* || !canConnectTo(*it)*/) {
            return false;
        }
    }
    return true;
}

bool Output::isConnected() const
{
    return targets_.size() > 0 || force_send_message_;
}

void Output::connectionMovePreview(Connectable *other_side)
{
    foreach(Input* input, targets_) {
        Q_EMIT(connectionInProgress(input, other_side));
    }
}

void Output::validateConnections()
{
    foreach(Input* target, targets_) {
        target->validateConnections();
    }
}

bool Output::canSendMessages() const
{
    foreach(Input* input, targets_) {
        bool blocked = /*input->isEnabled() && */input->isBlocked();
        if(blocked) {
            return false;
        }
    }
    return true;
}

void Output::forceSendMessage(bool force)
{
    force_send_message_ = force;
}
