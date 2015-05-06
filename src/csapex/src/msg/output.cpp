/// HEADER
#include <csapex/msg/output.h>

/// COMPONENT
#include <csapex/msg/input.h>
#include <csapex/model/connection.h>
#include <csapex/msg/output_transition.h>
#include <csapex/command/meta.h>
#include <csapex/command/delete_connection.h>
#include <csapex/utility/timer.h>
#include <csapex/utility/assert.h>
#include <csapex/msg/message_traits.h>

/// SYSTEM
#include <iostream>

using namespace csapex;

Output::Output(OutputTransition* transition, const UUID& uuid)
    : Connectable(uuid), transition_(transition),
      force_send_message_(false), state_(State::IDLE)
{
}

Output::Output(OutputTransition* transition, Unique* parent, int sub_id)
    : Connectable(parent, sub_id, "out"),  transition_(transition),
      force_send_message_(false), state_(State::IDLE)
{
}

Output::~Output()
{
}

OutputTransition* Output::getTransition() const
{
    return transition_;
}


void Output::setState(State s)
{
    state_ = s;
}

Output::State Output::getState() const
{
    return state_;
}

void Output::reset()
{
    clear();

    setSequenceNumber(0);
}

int Output::countConnections()
{
    return connections_.size();
}

std::vector<ConnectionWeakPtr> Output::getConnections() const
{
    return connections_;
}


void Output::addConnection(ConnectionWeakPtr connection)
{
    transition_->addConnection(connection);
    Connectable::addConnection(connection);
}

void Output::removeConnection(ConnectionWeakPtr connection)
{
    transition_->removeConnection(connection);
    Connectable::removeConnection(connection);
}

void Output::removeConnection(Connectable* other_side)
{
    for(std::vector<ConnectionWeakPtr>::iterator i = connections_.begin(); i != connections_.end();) {
        ConnectionPtr c = i->lock();
        if(c->to() == other_side) {
            other_side->removeConnection(this);

            i = connections_.erase(i);

            Q_EMIT connectionRemoved(this);
            return;

        } else {
            ++i;
        }
    }
}

Command::Ptr Output::removeConnectionCmd(Connection* connection) {
    Command::Ptr removeThis(new command::DeleteConnection(this, connection->to()));

    return removeThis;
}

Command::Ptr Output::removeAllConnectionsCmd()
{
    command::Meta::Ptr removeAll(new command::Meta("Remove All Connections"));

    for(ConnectionWeakPtr connection : connections_) {
        Command::Ptr removeThis(new command::DeleteConnection(this, connection.lock()->to()));
        removeAll->add(removeThis);
    }

    return removeAll;
}

void Output::removeAllConnectionsNotUndoable()
{
    for(std::vector<ConnectionWeakPtr>::iterator i = connections_.begin(); i != connections_.end();) {
        i->lock()->to()->removeConnection(this);
        i = connections_.erase(i);
    }

    Q_EMIT disconnected(this);
}

void Output::disable()
{
    Connectable::disable();
}

bool Output::isConnectionPossible(Connectable *other_side)
{
    if(!other_side->canInput()) {
        std::cerr << "cannot connect, other side can't input" << std::endl;
        return false;
    }
    if(!other_side->canConnectTo(this, false)) {
        std::cerr << "cannot connect, other side can't connect" << std::endl;
        return false;
    }

    return true;
}

bool Output::targetsCanBeMovedTo(Connectable* other_side) const
{
    foreach(ConnectionWeakPtr connection, connections_) {
        if(!connection.lock()->to()->canConnectTo(other_side, true)/* || !canConnectTo(*it)*/) {
            return false;
        }
    }
    return true;
}

bool Output::isConnected() const
{
    if(force_send_message_) {
        return true;
    }

    for(const auto& c : connections_) {
        auto connection = c.lock();
        if(connection->to()->isEnabled() && connection->isEstablished()) {
            return true;
        }
    }
    return false;
}

void Output::connectionMovePreview(Connectable *other_side)
{
    for(ConnectionWeakPtr connection : connections_) {
        Q_EMIT(connectionInProgress(connection.lock()->to(), other_side));
    }
}

void Output::validateConnections()
{
    for(ConnectionWeakPtr connection : connections_) {
        connection.lock()->to()->validateConnections();
    }
}

bool Output::canSendMessages() const
{
    for(ConnectionWeakPtr connection : connections_) {
        ConnectionPtr c = connection.lock();
        if(!c) {
            continue;
        }
    }
    return true;
}

void Output::forceSendMessage(bool force)
{
    force_send_message_ = force;
}
