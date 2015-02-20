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

protected:
    virtual void connectionAdded(Connection* connection);
    bool areConnections(Connection::State state) const;
    bool isConnection(Connection::State state) const;

protected:
    NodeWorker* node_;

    std::vector<ConnectionWeakPtr> connections_;
    mutable std::recursive_mutex sync;
};

}

#endif // TRANSITION_H

