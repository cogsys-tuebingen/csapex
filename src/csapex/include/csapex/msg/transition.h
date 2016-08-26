#ifndef TRANSITION_H
#define TRANSITION_H

/// COMPONENT
#include <csapex/model/connection.h>
#include <csapex/utility/delegate.h>

/// SYSTEM
#include <mutex>
#include <vector>

namespace csapex
{

class CSAPEX_EXPORT Transition
{
public:
    Transition(delegate::Delegate0<> activation_fn);
    Transition();

    virtual ~Transition();

    void setActivationFunction(delegate::Delegate0<> activation_fn);

    void addConnection(ConnectionPtr connection);
    void removeConnection(ConnectionPtr connection);

    void updateConnections();

    bool hasConnection() const;
    bool hasConnection(const ConnectionPtr& connection) const;
    bool hasConnection(Connection* connection) const;
    bool hasActiveConnection() const;

    virtual void reset();


    virtual bool isEnabled() const = 0;
    /*TODO: find better name*/ void checkIfEnabled();

    bool areAllConnections(Connection::State state) const;
    bool areAllConnections(Connection::State a, /*or*/ Connection::State b) const;
    bool areAllConnections(Connection::State a, /*or*/ Connection::State b, /*or*/ Connection::State c) const;
    bool isOneConnection(Connection::State state) const;

    std::vector<ConnectionPtr> getConnections() const;

protected:
    virtual void connectionAdded(Connection* connection);
    virtual void connectionRemoved(Connection* connection);

    void trackConnection(Connection* connection, const csapex::slim_signal::Connection& c);

protected:
    delegate::Delegate0<> activation_fn_;

    std::vector<ConnectionPtr> connections_;

    std::map<Connection*, std::vector<csapex::slim_signal::Connection>> signal_connections_;

    mutable std::recursive_mutex sync;
};

}

#endif // TRANSITION_H

