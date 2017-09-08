/// HEADER
#include <csapex/model/connector_remote.h>

/// PROJECT
#include <csapex/io/protcol/connector_requests.h>

using namespace csapex;


ConnectorRemote::ConnectorRemote(UUID uuid, ConnectableOwnerPtr owner,
                                 SessionPtr session, ConnectorPtr tmp_connector)
    : Connector(uuid, owner),
      Remote(session),
      tmp_connector_(tmp_connector)
{
    observe(tmp_connector_->enabled_changed, enabled_changed);

    observe(tmp_connector_->essential_changed, essential_changed);

    observe(tmp_connector_->disconnected, disconnected);
    observe(tmp_connector_->connectionStart, connectionStart);
    observe(tmp_connector_->connectionInProgress, connectionInProgress);

    observe(tmp_connector_->connection_added_to, connection_added_to);
    observe(tmp_connector_->connection_removed_to, connection_removed_to);

    observe(tmp_connector_->connection_added, connection_added);
    observe(tmp_connector_->connection_faded, connection_faded);

    observe(tmp_connector_->connectionEnabled, connectionEnabled);
    observe(tmp_connector_->message_processed, message_processed);
    observe(tmp_connector_->connectableError, connectableError);

    observe(tmp_connector_->typeChanged, typeChanged);
    observe(tmp_connector_->labelChanged, labelChanged);
}


void ConnectorRemote::connectionMovePreview(ConnectorPtr other_side)
{
    tmp_connector_->connectionMovePreview(other_side);
}

bool ConnectorRemote::isConnectedTo(const UUID &other) const
{
    return request<bool, ConnectorRequests>(ConnectorRequests::ConnectorRequestType::IsConnectedTo, getUUID().getAbsoluteUUID(), other);
}

/**
 * begin: generate getters
 **/
#define HANDLE_ACCESSOR(_enum, type, function) \
type ConnectorRemote::function() const\
{\
    return request<type, ConnectorRequests>(ConnectorRequests::ConnectorRequestType::_enum, getUUID().getAbsoluteUUID());\
}

#include <csapex/model/connector_remote_accessors.hpp>
/**
 * end: generate getters
 **/
