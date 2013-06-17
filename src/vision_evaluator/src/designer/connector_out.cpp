/// HEADER
#include "connector_out.h"

/// COMPONENT
#include "connector_in.h"
#include "design_board.h"

/// SYSTEM
#include <assert.h>
#include <boost/foreach.hpp>
#include <iostream>

using namespace vision_evaluator;

ConnectorOut::ConnectorOut(Box *parent, int sub_id)
    : Connector(parent, sub_id)
{
}

ConnectorOut::~ConnectorOut()
{
    BOOST_FOREACH(ConnectorIn* i, targets_) {
        i->removeConnection(this);
    }
    Q_EMIT connectionChanged();
}

void ConnectorOut::removeConnection(Connector* other_side)
{
    for(std::vector<ConnectorIn*>::iterator i = targets_.begin(); i != targets_.end();) {
        if(*i == other_side) {
            i = targets_.erase(i);
            Q_EMIT connectionChanged();

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
    Q_EMIT connectionChanged();
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
    Q_EMIT connectionChanged();

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

void ConnectorOut::publish(ConnectionType::Ptr message)
{
    if(targets_.size() == 1) {
        targets_[0]->inputMessage(message);
    } else if(targets_.size() > 1) {
        BOOST_FOREACH(ConnectorIn* i, targets_) {
            i->inputMessage(message->clone());
        }
    } else {
        return;
    }

    overlay_->showPublisherSignal(centerPoint());
}
