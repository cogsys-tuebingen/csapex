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


bool ConnectorRemote::canConnectTo(Connector* other_side, bool move) const
{
    return tmp_connector_->canConnectTo(other_side, move);
}
bool ConnectorRemote::targetsCanBeMovedTo(Connector* other_side) const
{
    return tmp_connector_->targetsCanBeMovedTo(other_side);
}
bool ConnectorRemote::isConnectionPossible(Connector* other_side)
{
    return tmp_connector_->isConnectionPossible(other_side);
}

void ConnectorRemote::connectionMovePreview(ConnectorPtr other_side)
{
    tmp_connector_->connectionMovePreview(other_side);
}
std::vector<ConnectionPtr> ConnectorRemote::getConnections() const
{
    return tmp_connector_->getConnections();
}


#define GENERATE_GETTER(type, function, _enum) \
type ConnectorRemote::function() const\
{\
    return request<type, ConnectorRequests>(ConnectorRequests::ConnectorRequestType::_enum, getUUID().getAbsoluteUUID());\
}

GENERATE_GETTER(int, getCount, GetCount)
GENERATE_GETTER(bool, canOutput, CanOutput)
GENERATE_GETTER(bool, canInput, CanInput)
GENERATE_GETTER(bool, isOutput, IsOutput)
GENERATE_GETTER(bool, isInput, IsInput)
GENERATE_GETTER(bool, isOptional, IsOptional)
GENERATE_GETTER(bool, isSynchronous, IsSynchronous)
GENERATE_GETTER(bool, isVirtual, IsVirtual)
GENERATE_GETTER(bool, isParameter, IsParameter)
GENERATE_GETTER(bool, isGraphPort, IsGraphPort)
GENERATE_GETTER(bool, isEssential, IsEssential)
GENERATE_GETTER(std::string, getLabel, GetLabel)
GENERATE_GETTER(ConnectorType, getConnectorType, GetConnectorType)
GENERATE_GETTER(ConnectorDescription, getDescription, GetDescription)
GENERATE_GETTER(bool, isEnabled, IsEnabled)
GENERATE_GETTER(int, sequenceNumber, GetSequenceNumber)
GENERATE_GETTER(int, countConnections, GetConnectionCount)
GENERATE_GETTER(bool, hasActiveConnection, HasActiveConnection)
GENERATE_GETTER(bool, isConnected, IsConnected)
GENERATE_GETTER(std::string, makeStatusString, MakeStatusString)

TokenData::ConstPtr ConnectorRemote::getType() const
{
//    TokenData res = request<TokenData, ConnectorRequests>(ConnectorRequests::ConnectorRequestType::GetType, getUUID().getAbsoluteUUID());
//    return std::make_shared<TokenData>(res);
    return tmp_connector_->getType();
}
