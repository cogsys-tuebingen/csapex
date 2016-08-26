/// HEADER
#include <csapex/msg/transition.h>

/// COMPONENT
#include <csapex/model/node_handle.h>
#include <csapex/model/connection.h>
#include <csapex/model/connectable.h>
#include <csapex/utility/assert.h>

/// SYSTEM
#include <iostream>

using namespace csapex;

Transition::Transition(delegate::Delegate0<> activation_fn)
    : activation_fn_(activation_fn)
{
}

Transition::Transition()
{
}

Transition::~Transition()
{

}

void Transition::setActivationFunction(delegate::Delegate0<> activation_fn)
{
    activation_fn_ = activation_fn;
}

void Transition::addConnection(ConnectionPtr connection)
{
    std::unique_lock<std::recursive_mutex> lock(sync);
    connections_.push_back(connection);
    lock.unlock();

    connectionAdded(connection.get());
}

void Transition::removeConnection(ConnectionPtr connection)
{
    std::unique_lock<std::recursive_mutex> lock(sync);
    for(auto it = connections_.begin(); it != connections_.end(); ++it) {
        if(*it == connection) {
            connections_.erase(it);
            break;
        }
    }
}

void Transition::reset()
{
}

void Transition::updateConnections()
{
}


void Transition::checkIfEnabled()
{
    //    if(isEnabled()) { // TODO: check here if enabled
    if(activation_fn_) {
        activation_fn_();
    }
    //    }
}

bool Transition::areAllConnections(Connection::State state) const
{
    std::unique_lock<std::recursive_mutex> lock(sync);
    for(ConnectionPtr connection : connections_) {
        if(connection->isEnabled() && connection->getState() != state) {
            return false;
        }
    }
    return true;
}

bool Transition::areAllConnections(Connection::State a, Connection::State b) const
{
    std::unique_lock<std::recursive_mutex> lock(sync);
    for(ConnectionPtr connection : connections_) {
        auto s = connection->getState();
        if(connection->isEnabled() && s != a && s != b) {
            return false;
        }
    }
    return true;
}

bool Transition::areAllConnections(Connection::State a, Connection::State b, Connection::State c) const
{
    std::unique_lock<std::recursive_mutex> lock(sync);
    for(ConnectionPtr connection : connections_) {
        auto s = connection->getState();
        if(connection->isEnabled() && s != a && s != b && s != c) {
            return false;
        }
    }
    return true;
}

bool Transition::isOneConnection(Connection::State state) const
{
    std::unique_lock<std::recursive_mutex> lock(sync);
    for(ConnectionPtr connection : connections_) {
        if(connection->isEnabled() && connection->getState() == state) {
            return true;
        }
    }
    return false;
}

bool Transition::hasConnection() const
{
    std::unique_lock<std::recursive_mutex> lock(sync);
    return !connections_.empty();
}
bool Transition::hasConnection(const ConnectionPtr& c) const
{
    return hasConnection(c.get());
}
bool Transition::hasConnection(Connection* connection) const
{
    std::unique_lock<std::recursive_mutex> lock(sync);
    for(const ConnectionPtr& c : connections_) {
        if(c.get() == connection) {
            return true;
        }
    }
    return false;
}
bool Transition::hasActiveConnection() const
{
    std::unique_lock<std::recursive_mutex> lock(sync);
    for(ConnectionPtr connection : connections_) {
        if(connection->isEnabled() && connection->isActive()) {
            return true;
        }
    }
    return false;
}


void Transition::connectionAdded(Connection */*connection*/)
{

}

void Transition::connectionRemoved(Connection *connection)
{
    for(auto& c : signal_connections_[connection]) {
        c.disconnect();
    }
    signal_connections_.erase(connection);

    checkIfEnabled();
}

void Transition::trackConnection(Connection *connection, const csapex::slim_signal::Connection &c)
{
    signal_connections_[connection].push_back(c);
}

std::vector<ConnectionPtr> Transition::getConnections() const
{
    return connections_;
}
