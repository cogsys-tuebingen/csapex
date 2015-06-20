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

    void addConnection(ConnectionPtr connection);
    void fadeConnection(ConnectionPtr connection);

    void updateConnections();

    bool hasEstablishedConnection() const;
    bool hasUnestablishedConnection() const;
    bool hasFadingConnection() const;

    void removeFadingConnections();

    virtual void establishConnections() = 0;
    virtual void reset() = 0;

protected:
    bool areAllConnections(Connection::State state) const;
    bool areAllConnections(Connection::State a, /*or*/ Connection::State b) const;
    bool areAllConnections(Connection::State a, /*or*/ Connection::State b, /*or*/ Connection::State c) const;
    bool isOneConnection(Connection::State state) const;

    void establishConnection(ConnectionPtr connection);

    virtual void connectionAdded(Connection* connection);
    virtual void connectionRemoved(Connection* connection);

    void trackConnection(Connection* connection, const boost::signals2::connection& c);

protected:
    NodeWorker* node_;

    std::vector<ConnectionPtr> established_connections_;
    std::vector<ConnectionPtr> unestablished_connections_;
    std::vector<ConnectionPtr> fading_connections_;

    std::map<Connection*, std::vector<boost::signals2::connection>> signal_connections_;

    mutable std::recursive_mutex sync;
};

}

#endif // TRANSITION_H

