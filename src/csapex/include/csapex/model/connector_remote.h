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
    ConnectorRemote(UUID uuid, ConnectableOwnerPtr owner, SessionPtr session,
                    ConnectorPtr tmp_connector);

    virtual bool isConnectedTo(const UUID &other) const override;
    virtual bool isActivelyConnectedTo(const UUID &other) const override;

    /**
     * begin: generate getters
     **/
    #define HANDLE_ACCESSOR(_enum, type, function) \
    virtual type function() const override;

    #include <csapex/model/connector_remote_accessors.hpp>
    #undef HANDLE_ACCESSOR
    /**
     * end: generate getters
     **/


private:
    ConnectorPtr tmp_connector_;
};

}

#endif // CONNECTOR_REMOTE_H
