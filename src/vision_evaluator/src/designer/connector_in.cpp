/// HEADER
#include "connector_in.h"

/// COMPONENT
#include "connector_out.h"

/// SYSTEM
#include <assert.h>
#include <iostream>

using namespace vision_evaluator;

ConnectorIn::ConnectorIn(Box* parent, int sub_id)
    : Connector(parent, sub_id), input(NULL)
{
}

ConnectorIn::~ConnectorIn()
{
    if(input != NULL) {
        input->removeConnection(this);
        Q_EMIT connectionChanged();
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
    Q_EMIT connectionChanged();
    return true;
}

void ConnectorIn::removeConnection(Connector* other_side)
{
    if(input != NULL) {
        assert(other_side == input);
        input = NULL;
        Q_EMIT connectionChanged();
    }
}

void ConnectorIn::removeAllConnections()
{
    if(input != NULL) {
        input->removeConnection(this);
        input = NULL;
        Q_EMIT disconnected(this);
        Q_EMIT connectionChanged();
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

void ConnectorIn::inputMessage(ConnectionType::Ptr message)
{
    message_ = message;

    Q_EMIT messageArrived(this);

    overlay_->showPublisherSignal(centerPoint());
}

ConnectionType::Ptr ConnectorIn::getMessage()
{
    return message_;
}
