#ifndef CONNECTOR_REMOTE_H
#define CONNECTOR_REMOTE_H

/// PROJECT
#include <csapex/model/connector.h>
#include <csapex/io/session.h>
#include <csapex/io/remote.h>


namespace csapex
{

class ConnectorRemote : public Connector, public Observer, public Remote
{
public:
    ConnectorRemote(const SessionPtr &session, UUID uuid, ConnectableOwnerPtr owner);
    ConnectorRemote(const SessionPtr &session, UUID uuid, ConnectableOwnerPtr owner, const ConnectorDescription& cd);

    virtual bool isConnectedTo(const UUID &other) const override;
    virtual bool isActivelyConnectedTo(const UUID &other) const override;

    /**
     * begin: generate getters
     **/
    #define HANDLE_ACCESSOR(_enum, type, function) \
    virtual type function() const override;

    #define HANDLE_STATIC_ACCESSOR(_enum, type, function) HANDLE_ACCESSOR(_enum, type, function)
    #define HANDLE_DYNAMIC_ACCESSOR(_enum, signal, type, function) HANDLE_ACCESSOR(_enum, type, function)

    #include <csapex/model/connector_remote_accessors.hpp>
    /**
     * end: generate getters
     **/


private:
    io::ChannelPtr channel_;

    /**
     * begin: generate caches
     **/
    #define HANDLE_ACCESSOR(_enum, type, function)
    #define HANDLE_STATIC_ACCESSOR(_enum, type, function) \
    mutable bool has_##function##_; \
    mutable type cache_##function##_;
    #define HANDLE_DYNAMIC_ACCESSOR(_enum, signal, type, function) \
    mutable bool has_##function##_; \
    mutable type value_##function##_;

    #include <csapex/model/connector_remote_accessors.hpp>
    #undef HANDLE_ACCESSOR
    /**
     * end: generate caches
     **/

    bool connected_;
};

}

#endif // CONNECTOR_REMOTE_H
