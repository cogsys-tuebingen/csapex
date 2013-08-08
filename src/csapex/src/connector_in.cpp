/// HEADER
#include <csapex/connector_in.h>

/// COMPONENT
#include <csapex/connector_out.h>
#include <csapex/command_delete_connection.h>

/// SYSTEM
#include <assert.h>
#include <iostream>

using namespace csapex;

ConnectorIn::ConnectorIn(Box* parent, int sub_id)
    : Connector(parent, "in", sub_id), input(NULL)
{
}

ConnectorIn::~ConnectorIn()
{
    if(input != NULL) {
        input->removeConnection(this);
    }
}

bool ConnectorIn::tryConnect(Connector* other_side)
{
    if(!other_side->isOutput()) {
        return false;
    }

    return other_side->tryConnect(this);
}

bool ConnectorIn::acknowledgeConnection(Connector* other_side)
{
    input = dynamic_cast<ConnectorOut*>(other_side);
    connect(other_side, SIGNAL(destroyed(QObject*)), this, SLOT(removeConnection(QObject*)));
    return true;
}

void ConnectorIn::removeConnection(Connector* other_side)
{
    if(input != NULL) {
        assert(other_side == input);
        input = NULL;
    }
}

Command::Ptr ConnectorIn::removeAllConnectionsCmd()
{
    Command::Ptr cmd(new command::DeleteConnection(input, this));
    return cmd;
}

void ConnectorIn::removeAllConnectionsNotUndoable()
{
    if(input != NULL) {
        input->removeConnection(this);
        input = NULL;
        setError(false);
        Q_EMIT disconnected(this);
    }
}

bool ConnectorIn::canConnect()
{
    return input == NULL;
}

bool ConnectorIn::targetsCanConnectTo(Connector* other_side)
{
    return getConnected()->canConnectTo(other_side) && canConnectTo(getConnected());
}

bool ConnectorIn::isConnected()
{
    return input != NULL;
}

void ConnectorIn::connectionMovePreview(Connector *other_side)
{
    Q_EMIT(connectionInProgress(getConnected(), other_side));
}


void ConnectorIn::validateConnections()
{
    bool e = false;
    if(isConnected()) {
        if(!input->getType()->canConnectTo(getType())) {
            e = true;
        }
    }

    setError(e);
}

ConnectorOut* ConnectorIn::getConnected()
{
    return input;
}

void ConnectorIn::inputMessage(ConnectionType::Ptr message)
{
    if(isError()) {
        return;
    }

    message_ = message;

    Q_EMIT messageArrived(this);
}

ConnectionType::Ptr ConnectorIn::getMessage()
{
    return message_;
}
