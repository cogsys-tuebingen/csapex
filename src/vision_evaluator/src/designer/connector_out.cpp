/// HEADER
#include "connector_out.h"

/// COMPONENT
#include "connector_in.h"

/// SYSTEM
#include <assert.h>
#include <iostream>

using namespace vision_evaluator;

ConnectorOut::ConnectorOut(QWidget* parent)
    : Connector(parent)
{
}

ConnectorOut::~ConnectorOut()
{
    for(std::vector<ConnectorIn*>::iterator i = targets_.begin(); i != targets_.end(); ++i) {
        (*i)->removeConnection(this);
    }
}

void ConnectorOut::removeConnection(Connector* other_side)
{
    for(std::vector<ConnectorIn*>::iterator i = targets_.begin(); i != targets_.end();) {
        if(*i == other_side) {
            i = targets_.erase(i);

        } else {
            ++i;
        }
    }
}

void ConnectorOut::removeAllConnections()
{
    for(std::vector<ConnectorIn*>::iterator i = targets_.begin(); i != targets_.end();) {
        (*i)->removeConnection(this);
        i = targets_.erase(i);
    }

    Q_EMIT disconnected(this);
}

bool ConnectorOut::tryConnect(Connector* other_side)
{
    if(!other_side->isInput()) {
        return false;
    }
    if(!other_side->canConnect()) {
        return false;
    }

    ConnectorIn* input = dynamic_cast<ConnectorIn*>(other_side);

    if(!input->acknowledgeConnection(this)) {
        return false;
    }

    targets_.push_back(input);

    connect(other_side, SIGNAL(destroyed(QObject*)), this, SLOT(removeConnection(QObject*)));

    overlay_->addConnection(this, input);

    return true;
}

bool ConnectorOut::canConnect()
{
    return true;
}
bool ConnectorOut::isConnected()
{
    return targets_.size() > 0;
}
