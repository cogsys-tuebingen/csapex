/// HEADER
#include <csapex/msg/transition.h>

/// COMPONENT
#include <csapex/model/node_worker.h>
#include <csapex/model/connection.h>
#include <csapex/model/connectable.h>
#include <csapex/utility/assert.h>

using namespace csapex;

Transition::Transition(NodeWorker *node)
    : node_(node)
{

}

Transition::~Transition()
{

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

void Transition::update()
{
    if(hasUnestablishedConnection()) {
        establish();
    }
    if(hasFadingConnection()) {
        removeFadingConnections();
    }
}

void Transition::establishConnection(ConnectionPtr connection)
{
    std::unique_lock<std::recursive_mutex> lock(sync);
    for(auto it = unestablished_connections_.begin(); it != unestablished_connections_.end(); ) {
        ConnectionPtr c = *it;
        if(connection == c) {
            if(!c->isEstablished()) {
                c->establish();
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

NodeWorker* Transition::getNode() const
{
    return node_;
}


void Transition::connectionAdded(Connection *connection)
{

}

void Transition::connectionRemoved(Connection *connection)
{

}
