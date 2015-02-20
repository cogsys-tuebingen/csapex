/// HEADER
#include <csapex/msg/transition.h>

/// COMPONENT
#include <csapex/model/node_worker.h>
#include <csapex/model/connection.h>

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
    std::lock_guard<std::recursive_mutex> lock(sync);
    connections_.push_back(connection);

    ConnectionPtr c = connection.lock();
    connectionAdded(c.get());
}

void Transition::connectionAdded(Connection *connection)
{

}



bool Transition::areConnections(Connection::State state) const
{
    for(ConnectionWeakPtr c : connections_) {
        ConnectionPtr connection = c.lock();
        if(connection->getState() != state) {
            return false;
        }
    }
    return true;
}

bool Transition::isConnection(Connection::State state) const
{
    for(ConnectionWeakPtr c : connections_) {
        ConnectionPtr connection = c.lock();
        if(connection->getState() == state) {
            return true;
        }
    }
    return false;
}

void Transition::removeConnection(ConnectionWeakPtr connection)
{
    std::lock_guard<std::recursive_mutex> lock(sync);
    for(auto it = connections_.begin(); it != connections_.end(); ) {
        const auto& c = *it;
        if(c.lock() == connection.lock()) {
            it = connections_.erase(it);
        } else {
            ++it;
        }
    }
}

NodeWorker* Transition::getNode() const
{
    return node_;
}
