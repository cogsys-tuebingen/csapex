/// HEADER
#include <csapex/model/connector_in.h>

/// COMPONENT
#include <csapex/model/box.h>
#include <csapex/model/connector_out.h>
#include <csapex/command/delete_connection.h>

/// SYSTEM
#include <assert.h>
#include <iostream>

using namespace csapex;

ConnectorIn::ConnectorIn(Box* parent, const std::string &uuid)
    : Connector(parent, uuid), target(NULL)
{
}

ConnectorIn::ConnectorIn(Box* parent, int sub_id)
    : Connector(parent, sub_id, TYPE_IN), target(NULL)
{
}

ConnectorIn::~ConnectorIn()
{
    if(target != NULL) {
        target->removeConnection(this);
    }
}

bool ConnectorIn::tryConnect(Connector* other_side)
{
    if(!other_side->canOutput()) {
        std::cerr << "cannot connect, other side can't output" << std::endl;
        return false;
    }

    return other_side->tryConnect(this);
}

bool ConnectorIn::acknowledgeConnection(Connector* other_side)
{
    target = dynamic_cast<ConnectorOut*>(other_side);
    connect(other_side, SIGNAL(destroyed(QObject*)), this, SLOT(removeConnection(QObject*)));
    return true;
}

void ConnectorIn::removeConnection(Connector* other_side)
{
    if(target != NULL) {
        assert(other_side == target);
        target = NULL;
    }
}

Command::Ptr ConnectorIn::removeAllConnectionsCmd()
{
    Command::Ptr cmd(new command::DeleteConnection(target, this));
    return cmd;
}

void ConnectorIn::removeAllConnectionsNotUndoable()
{
    if(target != NULL) {
        target->removeConnection(this);
        target = NULL;
        setError(false);
        Q_EMIT disconnected(this);
    }
}


bool ConnectorIn::canConnectTo(Connector* other_side, bool move) const
{
    return Connector::canConnectTo(other_side, move) && (move || !isConnected());
}

bool ConnectorIn::targetsCanBeMovedTo(Connector* other_side) const
{
    return target->canConnectTo(other_side, true) /*&& canConnectTo(getConnected())*/;
}

bool ConnectorIn::isConnected() const
{
    return target != NULL;
}

void ConnectorIn::connectionMovePreview(Connector *other_side)
{
    Q_EMIT(connectionInProgress(getConnected(), other_side));
}


void ConnectorIn::validateConnections()
{
    bool e = false;
    if(isConnected()) {
        if(!target->getType()->canConnectTo(getType(), true)) {
            e = true;
        }
    }

    setError(e);
}

Connector *ConnectorIn::getConnected() const
{
    return target;
}

void ConnectorIn::inputMessage(ConnectionType::Ptr message)
{
    if(isError()) {
        return;
    }

    {
        QMutexLocker lock(&io_mutex);
        message_ = message;
    }

    count_++;

    Q_EMIT messageArrived(this);
}

ConnectionType::Ptr ConnectorIn::getMessage()
{
    QMutexLocker lock(&io_mutex);
    return message_;
}
