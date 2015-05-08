#ifndef TRANSITION_H
#define TRANSITION_H

/// COMPONENT
#include <csapex/csapex_fwd.h>
#include <csapex/model/connection.h>

/// SYSTEM
#include <mutex>
#include <vector>

namespace csapex
{

class Transition
{
public:
    Transition(NodeWorker* node);
    virtual ~Transition();

    NodeWorker* getNode() const;

    void addConnection(ConnectionWeakPtr connection);
    void removeConnection(ConnectionWeakPtr connection);

    bool hasUnestablishedConnection() const;

protected:
    virtual void connectionAdded(Connection* connection);
    bool areConnections(Connection::State state) const;
    bool areConnections(Connection::State a, /*or*/ Connection::State b) const;
    bool areConnections(Connection::State a, /*or*/ Connection::State b, /*or*/ Connection::State c) const;
    bool isConnection(Connection::State state) const;

    void establishConnection(ConnectionWeakPtr connection);

protected:
    NodeWorker* node_;

    std::vector<ConnectionWeakPtr> established_connections_;
    std::vector<ConnectionWeakPtr> unestablished_connections_;
    mutable std::recursive_mutex sync;
};

}

#endif // TRANSITION_H

