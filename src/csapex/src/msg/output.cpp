/// HEADER
#include <csapex/msg/output.h>

/// COMPONENT
#include <csapex/msg/input.h>
#include <csapex/model/connection.h>
#include <csapex/profiling/timer.h>
#include <csapex/utility/assert.h>
#include <csapex/msg/token_traits.h>
#include <csapex/utility/debug.h>

/// SYSTEM
#include <iostream>
#include <sstream>

using namespace csapex;

Output::Output(const UUID& uuid, ConnectableOwnerWeakPtr owner)
    : Connectable(uuid, owner), transition_(nullptr),
      state_(State::IDLE)
{
}

Output::~Output()
{
}

void Output::setOutputTransition(OutputTransition *ot)
{
    transition_ = ot;
}

void Output::removeOutputTransition()
{
    transition_ = nullptr;
}

void Output::removeConnection(Connector *other_side)
{
    Connectable::removeConnection(other_side);
    if(connections_.empty()) {
        setState(State::IDLE);
    }
}

void Output::notifyMessageProcessed()
{
    //TRACE std::cout << getUUID() << " notified" << std::endl;
    setState(State::IDLE);
    message_processed(shared_from_this());
}

void Output::notifyMessageProcessed(Connection* connection)
{
    for(auto connection : connections_) {
        if(connection->getState() != Connection::State::DONE) {
            return;
        }
    }

    //TRACE std::cout << getUUID() << " is processed" << std::endl;

    notifyMessageProcessed();
}

void Output::activate()
{
    setState(State::ACTIVE);
}

void Output::setState(State s)
{
    if(state_ == s) {
        return;
    }
    if(s == State::IDLE) {
        //TRACE std::cout << "output " << getUUID() << " is now idle" << std::endl;
    } else {
        //TRACE std::cout << "output " << getUUID() << " is now active" << std::endl;
    }
    state_ = s;
}

Output::State Output::getState() const
{
    return state_;
}

void Output::reset()
{
    Connectable::reset();

    clearBuffer();

    setSequenceNumber(0);
    setState(State::IDLE);
}

std::vector<ConnectionPtr> Output::getConnections() const
{
    return connections_;
}

void Output::removeAllConnectionsNotUndoable()
{
    std::unique_lock<std::recursive_mutex> lock(sync_mutex);
    for(std::vector<ConnectionPtr>::iterator i = connections_.begin(); i != connections_.end();) {
        (*i)->to()->removeConnection(this);
        i = connections_.erase(i);
    }

    disconnected(shared_from_this());
}

void Output::disable()
{
    Connectable::disable();
}

bool Output::isConnectionPossible(Connector *other_side)
{
    if(!other_side->canInput()) {
        std::cerr << "cannot connect " << getUUID() << " to " << other_side->getUUID() << ", other side can't input" << std::endl;
        return false;
    }
    if(!other_side->canConnectTo(this, false)) {
        std::cerr << "cannot connect " << getUUID() << " to " << other_side->getUUID() << ", not compatible" << std::endl;
        return false;
    }

    return true;
}

bool Output::targetsCanBeMovedTo(Connector* other_side) const
{
    std::unique_lock<std::recursive_mutex> lock(sync_mutex);
    for(ConnectionPtr connection : connections_) {
        if(!connection->to()->canConnectTo(other_side, true)/* || !canConnectTo(*it)*/) {
            return false;
        }
    }
    return true;
}

bool Output::isConnected() const
{
    return !connections_.empty();
}

void Output::connectionMovePreview(ConnectorPtr other_side)
{
    std::unique_lock<std::recursive_mutex> lock(sync_mutex);
    for(ConnectionPtr connection : connections_) {
        connectionInProgress(connection->to(), other_side);
    }
}

void Output::validateConnections()
{
    for(ConnectionPtr connection : connections_) {
        connection->to()->validateConnections();
    }
}

bool Output::canReceiveToken() const
{
    for(const ConnectionPtr& connection : connections_) {
        if(connection->getState() != Connection::State::NOT_INITIALIZED) {
            return false;
        }
    }
    return true;
}

bool Output::canSendMessages() const
{
    for(const ConnectionPtr& connection : connections_) {
        if(connection->getState() == Connection::State::NOT_INITIALIZED) {
            return false;
        }
    }
    return true;
}

void Output::publish()
{
    apex_assert_hard(isEnabled());
    auto msg = getToken();
    apex_assert_hard(msg);

    std::unique_lock<std::recursive_mutex> lock(sync_mutex);
    for(auto connection : connections_) {
        if(connection->isEnabled()) {
            connection->setToken(msg);
        }
    }
}

void Output::addStatusInformation(std::stringstream &status_stream) const
{
    status_stream << ", state: ";
    switch(getState()) {
    case Output::State::ACTIVE:
        status_stream << "ACTIVE";
        break;
    case Output::State::IDLE:
        status_stream << "IDLE";
        break;
    default:
        status_stream << "UNKNOWN";
    }
}
