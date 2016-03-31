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
    unestablished_connections_.push_back(connection);
    lock.unlock();

    connectionAdded(connection.get());
}

void Transition::fadeConnection(ConnectionPtr connection)
{
    std::unique_lock<std::recursive_mutex> lock(sync);
    bool contained = false;
    for(auto it = established_connections_.begin(); it != established_connections_.end(); ++it) {
        if(*it == connection) {
            contained = true;
            break;
        }
    }

    if(contained) {
        fading_connections_.push_back(connection);
    } else {
        // connection is not yet established but should already be deleted?
        for(auto it = unestablished_connections_.begin(); it != unestablished_connections_.end(); ++it) {
            if(*it == connection) {
                unestablished_connections_.erase(it);
                break;
            }
        }
    }
}

void Transition::reset()
{
    removeFadingConnections();
    establishConnections();
}

void Transition::removeFadingConnections()
{
    std::unique_lock<std::recursive_mutex> lock(sync);
    for(ConnectionPtr connection : fading_connections_) {
        for(auto it = established_connections_.begin(); it != established_connections_.end(); ) {
            const auto& c = *it;
            if(c == connection) {
                it = established_connections_.erase(it);
                connectionRemoved(connection.get());

            } else {
                ++it;
            }
        }
    }
    fading_connections_.clear();
}

void Transition::updateConnections()
{
    if(areAllConnections(Connection::State::DONE, Connection::State::READ)) {
        if(hasUnestablishedConnection()) {
            establishConnections();
        }
        if(hasFadingConnection()) {
            removeFadingConnections();
        }
    }
}



void Transition::checkIfEnabled()
{
    //    if(isEnabled()) { // TODO: check here if enabled
    if(activation_fn_) {
        activation_fn_();
    }
    //    }
}

void Transition::establishConnection(ConnectionPtr connection)
{
    apex_assert_hard(areAllConnections(Connection::State::DONE, Connection::State::READ));

    std::unique_lock<std::recursive_mutex> lock(sync);
    for(auto it = unestablished_connections_.begin(); it != unestablished_connections_.end(); ) {
        ConnectionPtr c = *it;
        if(connection == c) {
            if(c->isSourceEstablished() && c->isSinkEstablished()) {
                if(!c->isEstablished()) {
                    c->establish();
                }
            }

            it = unestablished_connections_.erase(it);
            established_connections_.push_back(c);
        } else {
            ++it;
        }
    }
}


bool Transition::areAllConnections(Connection::State state) const
{
    std::unique_lock<std::recursive_mutex> lock(sync);
    for(ConnectionPtr connection : established_connections_) {
        if(connection->isSinkEnabled() && connection->getState() != state) {
            return false;
        }
    }
    return true;
}

bool Transition::areAllConnections(Connection::State a, Connection::State b) const
{
    std::unique_lock<std::recursive_mutex> lock(sync);
    for(ConnectionPtr connection : established_connections_) {
        auto s = connection->getState();
        if(connection->isSinkEnabled() && s != a && s != b) {
            return false;
        }
    }
    return true;
}

bool Transition::areAllConnections(Connection::State a, Connection::State b, Connection::State c) const
{
    std::unique_lock<std::recursive_mutex> lock(sync);
    for(ConnectionPtr connection : established_connections_) {
        auto s = connection->getState();
        if(connection->isSinkEnabled() &&  s != a && s != b && s != c) {
            return false;
        }
    }
    return true;
}

bool Transition::isOneConnection(Connection::State state) const
{
    std::unique_lock<std::recursive_mutex> lock(sync);
    for(ConnectionPtr connection : established_connections_) {
        if(connection->isSinkEnabled() &&  connection->getState() == state) {
            return true;
        }
    }
    return false;
}

bool Transition::hasEstablishedConnection() const
{
    std::unique_lock<std::recursive_mutex> lock(sync);
    return !established_connections_.empty();
}

bool Transition::hasUnestablishedConnection() const
{
    std::unique_lock<std::recursive_mutex> lock(sync);
    return !unestablished_connections_.empty();
}

bool Transition::hasFadingConnection() const
{
    std::unique_lock<std::recursive_mutex> lock(sync);
    return !fading_connections_.empty();
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

std::vector<ConnectionPtr> Transition::getEstablishedConnections() const
{
    return established_connections_;
}


std::vector<ConnectionPtr> Transition::getFadingConnections() const
{
    return fading_connections_;
}
