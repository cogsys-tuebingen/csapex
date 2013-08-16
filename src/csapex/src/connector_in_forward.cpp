/// HEADER
#include <csapex/connector_in_forward.h>

using namespace csapex;

ConnectorInForward::ConnectorInForward(Box* parent, const std::string &uuid)
    : ConnectorIn(parent, uuid)
{
}

ConnectorInForward::ConnectorInForward(Box* parent, int sub_id)
    : ConnectorIn(parent, sub_id)
{
}


ConnectorInForward::~ConnectorInForward()
{
    if(target != NULL) {
        target->removeConnection(this);
    }
}

bool ConnectorInForward::isForwarding() const
{
    return true;
}

void ConnectorInForward::inputMessage(ConnectionType::Ptr message)
{
    ConnectorIn::inputMessage(message);
}

bool ConnectorInForward::tryConnect(Connector* other_side)
{
    if(!other_side->isInput()) {
        return false;
    }

    return other_side->tryConnect(this);
}

bool ConnectorInForward::acknowledgeConnection(Connector* other_side)
{
    target = dynamic_cast<ConnectorIn*>(other_side);
    connect(other_side, SIGNAL(destroyed(QObject*)), this, SLOT(removeConnection(QObject*)));
    return true;
}

void ConnectorInForward::removeConnection(Connector* other_side)
{
    if(target != NULL) {
        assert(other_side == target);
        target = NULL;
    }
}

void ConnectorInForward::removeAllConnectionsNotUndoable()
{
    if(target != NULL) {
        target->removeConnection(this);
        target = NULL;
        setError(false);
        Q_EMIT disconnected(this);
    }
}

bool ConnectorInForward::canConnect()
{
    return target == NULL;
}

bool ConnectorInForward::isConnected()
{
    return target != NULL;
}

void ConnectorInForward::validateConnections()
{
    bool e = false;
    if(isConnected()) {
        if(!target->getType()->canConnectTo(getType())) {
            e = true;
        }
    }

    setError(e);
}
