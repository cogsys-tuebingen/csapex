/// HEADER
#include "connector_in.h"

/// COMPONENT
#include "connector_out.h"
#include "command_delete_connection.h"

/// SYSTEM
#include <assert.h>
#include <iostream>

using namespace vision_evaluator;

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
        Q_EMIT disconnected(this);
    }
}

bool ConnectorIn::canConnect()
{
    return input == NULL;
}

bool ConnectorIn::isConnected()
{
    return input != NULL;
}

ConnectorOut* ConnectorIn::getConnected()
{
    return input;
}

void ConnectorIn::inputMessage(ConnectionType::Ptr message)
{
    message_ = message;

    Q_EMIT messageArrived(this);
}

ConnectionType::Ptr ConnectorIn::getMessage()
{
    return message_;
}
