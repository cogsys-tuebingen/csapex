/// HEADER
#include <csapex/connector_out.h>

/// COMPONENT
#include <csapex/connector_in.h>
#include <csapex/design_board.h>
#include <csapex/command_meta.h>
#include <csapex/command_delete_connection.h>

/// SYSTEM
#include <assert.h>
#include <boost/foreach.hpp>
#include <iostream>

using namespace csapex;

ConnectorOut::ConnectorOut(Box* parent, int sub_id)
    : Connector(parent, "out", sub_id)
{
}

ConnectorOut::~ConnectorOut()
{
    BOOST_FOREACH(ConnectorIn* i, targets_) {
        i->removeConnection(this);
        Q_EMIT connectionDestroyed(this, i);
    }
}

ConnectorOut::TargetIterator ConnectorOut::beginTargets()
{
    return targets_.begin();
}
ConnectorOut::TargetIterator ConnectorOut::endTargets()
{
    return targets_.end();
}

void ConnectorOut::removeConnection(Connector* other_side)
{
    for(std::vector<ConnectorIn*>::iterator i = targets_.begin(); i != targets_.end();) {
        if(*i == other_side) {
            other_side->removeConnection(this);
            Q_EMIT connectionDestroyed(this, *i);

            i = targets_.erase(i);
        } else {
            ++i;
        }
    }
}

Command::Ptr ConnectorOut::removeConnectionCmd(ConnectorIn* other_side) {
    Command::Ptr removeThis(new command::DeleteConnection(this, other_side));

    return removeThis;
}

Command::Ptr ConnectorOut::removeAllConnectionsCmd()
{
    command::Meta::Ptr removeAll(new command::Meta);

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
        Q_EMIT connectionDestroyed(this, *i);
        i = targets_.erase(i);
    }

    Q_EMIT disconnected(this);
}

void ConnectorOut::connectForcedWithoutCommand(ConnectorIn *other_side)
{
    tryConnect(other_side);
}

bool ConnectorOut::tryConnect(Connector* other_side)
{
    if(!other_side->isInput()) {
        return false;
    }
    if(!other_side->canConnect()) {
        return false;
    }

    ConnectorIn* input = dynamic_cast<ConnectorIn*>(other_side);

    if(!input->acknowledgeConnection(this)) {
        return false;
    }

    targets_.push_back(input);

    connect(other_side, SIGNAL(destroyed(QObject*)), this, SLOT(removeConnection(QObject*)));

    Q_EMIT connectionFormed(this, input);

    validateConnections();

    return true;
}

bool ConnectorOut::canConnect()
{
    return true;
}

bool ConnectorOut::targetsCanConnectTo(Connector* other_side)
{
    for(ConnectorOut::TargetIterator it = beginTargets(); it != endTargets(); ++it) {
        if(!(*it)->canConnectTo(other_side) || !canConnectTo(*it)) {
            return false;
        }
    }
    return true;
}

bool ConnectorOut::isConnected()
{
    return targets_.size() > 0;
}

void ConnectorOut::connectionMovePreview(Connector *other_side)
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
    if(targets_.size() == 1) {
        targets_[0]->inputMessage(message);
    } else if(targets_.size() > 1) {
        BOOST_FOREACH(ConnectorIn* i, targets_) {
            i->inputMessage(message->clone());
        }
    } else {
        return;
    }

    Q_EMIT messageSent(this);
}
