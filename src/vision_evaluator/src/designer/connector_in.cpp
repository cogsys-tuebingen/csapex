/// HEADER
#include "connector_in.h"

/// COMPONENT
#include "connector_out.h"

/// SYSTEM
#include <assert.h>
#include <iostream>

ConnectorIn::ConnectorIn(QWidget* parent)
    : Connector(parent), input(NULL)
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

bool ConnectorIn::canConnect()
{
    return input == NULL;
}

bool ConnectorIn::isConnected()
{
    return input != NULL;
}
