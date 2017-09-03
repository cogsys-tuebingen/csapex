/// HEADER
#include <csapex/model/connector_remote.h>

using namespace csapex;

ConnectorRemote::ConnectorRemote(UUID uuid, ConnectableOwnerPtr owner,
                                 SessionPtr session, ConnectorPtr tmp_connector)
    : Connector(uuid, owner),
      session_(session),
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
void ConnectorRemote::validateConnections()
{
    tmp_connector_->validateConnections();
}
void ConnectorRemote::connectionMovePreview(ConnectorPtr other_side)
{
    tmp_connector_->connectionMovePreview(other_side);
}

int ConnectorRemote::getCount() const
{
    return tmp_connector_->getCount();
}

bool ConnectorRemote::canOutput() const
{
    return tmp_connector_->canOutput();
}
bool ConnectorRemote::canInput() const
{
    return tmp_connector_->canInput();
}
bool ConnectorRemote::isOutput() const
{
    return tmp_connector_->isOutput();
}
bool ConnectorRemote::isInput() const
{
    return tmp_connector_->isInput();
}
bool ConnectorRemote::isOptional() const
{
    return tmp_connector_->isOptional();
}
bool ConnectorRemote::isSynchronous() const
{
    return tmp_connector_->isSynchronous();
}

bool ConnectorRemote::isVirtual() const
{
    return tmp_connector_->isVirtual();
}

bool ConnectorRemote::isParameter() const
{
    return tmp_connector_->isParameter();
}

void ConnectorRemote::setVirtual(bool _virtual)
{
    tmp_connector_->setVirtual(_virtual);
}

bool ConnectorRemote::isGraphPort() const
{
    return tmp_connector_->isGraphPort();
}
void ConnectorRemote::setGraphPort(bool graph)
{
    tmp_connector_->setGraphPort(graph);
}

bool ConnectorRemote::isEssential() const
{
    return tmp_connector_->isEssential();
}
void ConnectorRemote::setEssential(bool essential)
{
    tmp_connector_->setEssential(essential);
}

void ConnectorRemote::addConnection(ConnectionPtr connection)
{
    tmp_connector_->addConnection(connection);
}
void ConnectorRemote::removeConnection(Connector* other_side)
{
    tmp_connector_->removeConnection(other_side);
}
void ConnectorRemote::fadeConnection(ConnectionPtr connection)
{
    tmp_connector_->fadeConnection(connection);
}

void ConnectorRemote::setLabel(const std::string& label)
{
    tmp_connector_->setLabel(label);
}
std::string ConnectorRemote::getLabel() const
{
    return tmp_connector_->getLabel();
}

void ConnectorRemote::setType(TokenData::ConstPtr type)
{
    tmp_connector_->setType(type);
}
TokenData::ConstPtr ConnectorRemote::getType() const
{
    return tmp_connector_->getType();
}

ConnectorType ConnectorRemote::getConnectorType() const
{
    return tmp_connector_->getConnectorType();
}

ConnectorDescription ConnectorRemote::getDescription() const
{
    return tmp_connector_->getDescription();
}

bool ConnectorRemote::isEnabled() const
{
    return tmp_connector_->isEnabled();
}

void ConnectorRemote::setEnabled(bool enabled)
{
    tmp_connector_->setEnabled(enabled);
}

int ConnectorRemote::sequenceNumber() const
{
    return tmp_connector_->sequenceNumber();
}
void ConnectorRemote::setSequenceNumber(int seq_no)
{
    tmp_connector_->setSequenceNumber(seq_no);
}

int ConnectorRemote::countConnections()
{
    return tmp_connector_->countConnections();
}
std::vector<ConnectionPtr> ConnectorRemote::getConnections() const
{
    return tmp_connector_->getConnections();
}

bool ConnectorRemote::hasActiveConnection() const
{
    return tmp_connector_->hasActiveConnection();
}

bool ConnectorRemote::isConnected() const
{
    return tmp_connector_->isConnected();
}

void ConnectorRemote::disable()
{
    tmp_connector_->disable();
}
void ConnectorRemote::enable()
{
    tmp_connector_->enable();
}

void ConnectorRemote::reset()
{
    tmp_connector_->reset();
}
void ConnectorRemote::stop()
{
    tmp_connector_->stop();
}

void ConnectorRemote::notifyMessageProcessed()
{
    tmp_connector_->notifyMessageProcessed();
}

std::string ConnectorRemote::makeStatusString() const
{
    return tmp_connector_->makeStatusString();
}
