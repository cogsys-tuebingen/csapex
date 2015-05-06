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
void Transition::addConnection(ConnectionWeakPtr connection)
{
    std::unique_lock<std::recursive_mutex> lock(sync);
    unestablished_connections_.push_back(connection);

    ConnectionPtr c = connection.lock();
    lock.unlock();

    connectionAdded(c.get());
}

void Transition::connectionAdded(Connection *connection)
{

}

void Transition::establishConnection(ConnectionWeakPtr connection)
{
    std::lock_guard<std::recursive_mutex> lock(sync);
    for(auto it = unestablished_connections_.begin(); it != unestablished_connections_.end(); ) {
        ConnectionWeakPtr c = *it;
        if(connection.lock() == c.lock()) {
            auto cptr = c.lock();
            if(!cptr->isEstablished()) {
                cptr->establish();
            }

            it = unestablished_connections_.erase(it);
            established_connections_.push_back(cptr);
        } else {
            ++it;
        }
    }
}


bool Transition::areConnections(Connection::State state) const
{
    std::lock_guard<std::recursive_mutex> lock(sync);
    for(ConnectionWeakPtr c : established_connections_) {
        ConnectionPtr connection = c.lock();
        if(connection->isEnabled() && connection->getState() != state) {
            return false;
        }
    }
    return true;
}
bool Transition::areConnections(Connection::State a, Connection::State b) const
{
    std::lock_guard<std::recursive_mutex> lock(sync);
    for(ConnectionWeakPtr c : established_connections_) {
        ConnectionPtr connection = c.lock();
        auto s = connection->getState();
        if(connection->isEnabled() && s != a && s != b) {
            return false;
        }
    }
    return true;
}

bool Transition::isConnection(Connection::State state) const
{
    std::lock_guard<std::recursive_mutex> lock(sync);
    for(ConnectionWeakPtr c : established_connections_) {
        ConnectionPtr connection = c.lock();
        if(connection->isEnabled() && connection->getState() == state) {
            return true;
        }
    }
    return false;
}

void Transition::removeConnection(ConnectionWeakPtr connection)
{
    std::lock_guard<std::recursive_mutex> lock(sync);
    for(auto it = established_connections_.begin(); it != established_connections_.end(); ) {
        const auto& c = *it;
        if(c.lock() == connection.lock()) {
            it = established_connections_.erase(it);
        } else {
            ++it;
        }
    }
}

bool Transition::hasUnestablishedConnection() const
{
    std::unique_lock<std::recursive_mutex> lock(sync);
    return !unestablished_connections_.empty();
}

NodeWorker* Transition::getNode() const
{
    return node_;
}
