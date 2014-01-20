/// HEADER
#include <csapex/model/connector_out.h>

/// COMPONENT
#include <csapex/model/connector_in.h>
#include <csapex/view/design_board.h>
#include <csapex/command/meta.h>
#include <csapex/command/delete_connection.h>
#include <csapex/view/port.h>

/// SYSTEM
#include <assert.h>
#include <boost/foreach.hpp>
#include <iostream>

using namespace csapex;

ConnectorOut::ConnectorOut(const UUID& uuid)
    : Connectable(uuid), force_send_message_(false)
{
}

ConnectorOut::ConnectorOut(Unique* parent, int sub_id)
    : Connectable(parent, sub_id, TYPE_OUT), force_send_message_(false)
{
}

ConnectorOut::~ConnectorOut()
{
    BOOST_FOREACH(ConnectorIn* i, targets_) {
        i->removeConnection(this);
    }
}

int ConnectorOut::noTargets()
{
    return targets_.size();
}

ConnectorOut::TargetIterator ConnectorOut::beginTargets() const
{
    return targets_.begin();
}
ConnectorOut::TargetIterator ConnectorOut::endTargets() const
{
    return targets_.end();
}

void ConnectorOut::removeConnection(Connectable* other_side)
{
    for(std::vector<ConnectorIn*>::iterator i = targets_.begin(); i != targets_.end();) {
        if(*i == other_side) {
            other_side->removeConnection(this);

            i = targets_.erase(i);

            Q_EMIT connectionRemoved();
            return;

        } else {
            ++i;
        }
    }
}

void ConnectorOut::updateIsProcessing()
{
    if(!isConnected()) {
        setProcessing(false);
    }
}

Command::Ptr ConnectorOut::removeConnectionCmd(ConnectorIn* other_side) {
    Command::Ptr removeThis(new command::DeleteConnection(this, other_side));

    return removeThis;
}

Command::Ptr ConnectorOut::removeAllConnectionsCmd()
{
    command::Meta::Ptr removeAll(new command::Meta("Remove All Connections"));

    BOOST_FOREACH(ConnectorIn* target, targets_) {
        Command::Ptr removeThis(new command::DeleteConnection(this, target));
        removeAll->add(removeThis);
    }

    return removeAll;
}

void ConnectorOut::removeAllConnectionsNotUndoable()
{
    for(std::vector<ConnectorIn*>::iterator i = targets_.begin(); i != targets_.end();) {
        (*i)->removeConnection(this);
        i = targets_.erase(i);
    }

    Q_EMIT disconnected(this);
}

void ConnectorOut::connectForcedWithoutCommand(ConnectorIn *other_side)
{
    tryConnect(other_side);
}

bool ConnectorOut::tryConnect(Connectable *other_side)
{
    return connect(other_side);
}

bool ConnectorOut::connect(Connectable *other_side)
{
    if(!other_side->canInput()) {
        std::cerr << "cannot connect, other side can't input" << std::endl;
        return false;
    }
    if(!other_side->canConnectTo(this, false)) {
        std::cerr << "cannot connect, other side can't connect" << std::endl;
        return false;
    }

    ConnectorIn* input = dynamic_cast<ConnectorIn*>(other_side);

    if(!input->acknowledgeConnection(this)) {
        std::cerr << "cannot connect, other side doesn't acknowledge" << std::endl;
        return false;
    }

    targets_.push_back(input);

    QObject::connect(other_side, SIGNAL(destroyed(QObject*)), this, SLOT(removeConnection(QObject*)));

    validateConnections();

    return true;
}

bool ConnectorOut::targetsCanBeMovedTo(Connectable* other_side) const
{
    for(ConnectorOut::TargetIterator it = beginTargets(); it != endTargets(); ++it) {
        if(!(*it)->canConnectTo(other_side, true)/* || !canConnectTo(*it)*/) {
            return false;
        }
    }
    return true;
}

bool ConnectorOut::isConnected() const
{
    return targets_.size() > 0;
}

void ConnectorOut::connectionMovePreview(Connectable *other_side)
{
    for(ConnectorOut::TargetIterator it = beginTargets(); it != endTargets(); ++it) {
        Q_EMIT(connectionInProgress(*it, other_side));
    }
}

void ConnectorOut::validateConnections()
{
    BOOST_FOREACH(ConnectorIn* target, targets_) {
        target->validateConnections();
    }
}

void ConnectorOut::publish(ConnectionType::Ptr message)
{
    // update buffer
    message_ = message;


    setProcessing(true);

    bool one_is_async = false;
    BOOST_FOREACH(ConnectorIn* i, targets_) {
        if(i->isAsync()) {
            one_is_async = true;
        }
    }


    // wait for all connected inputs to be able to receive, if none is async
    //  * inputs can only be connected to this output since they are 1:1
    std::vector<ConnectorIn*> targets;
    BOOST_FOREACH(ConnectorIn* i, targets_) {
        if(i->isEnabled()) {
            if(i->isProcessing() && !one_is_async) {
                setBlocked(true);
                i->waitForProcessing(getUUID());
            }
            targets.push_back(i);
        }
    }

    if(isBlocked()) {
        setBlocked(false);
    }

    if(targets.empty()) {
        setProcessing(false);
        return;
    }

    // all connected inputs are ready to receive, send them the message
    if(targets.size() == 1) {
        targets[0]->inputMessage(message_);
    } else if(targets.size() > 1) {
        BOOST_FOREACH(ConnectorIn* i, targets) {
            i->inputMessage(message_->clone());
        }
    }/* else if(!force_send_message_){
        return;
    }*/

    count_++;
    Q_EMIT messageSent(this);
}

void ConnectorOut::forceSendMessage(bool force)
{
    force_send_message_ = force;
}

ConnectionType::Ptr ConnectorOut::getMessage()
{
    return message_;
}
