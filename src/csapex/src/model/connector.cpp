/// HEADER
#include <csapex/model/connector.h>

/// PROJECT
#include <csapex/model/connectable_owner.h>
#include <csapex/model/connection.h>

using namespace csapex;

Connector::Connector(const UUID &uuid, ConnectableOwnerWeakPtr owner)
    : Unique(uuid),
      owner_(owner)
{

}

void Connector::validateConnections()
{

}

ConnectorDescription Connector::getDescription() const
{
    ConnectorDescription res(getOwner()->getUUID().getAbsoluteUUID(), getUUID(), getConnectorType(), getType(), getLabel(), isOptional());
    for(const ConnectionPtr& c : getConnections()) {
        if(isOutput()) {
            res.targets.push_back(c->target()->getUUID().getAbsoluteUUID());
        } else {
            res.targets.push_back(c->source()->getUUID().getAbsoluteUUID());
        }
    }
    return res;
}


ConnectableOwnerPtr Connector::getOwner() const
{
    return owner_.lock();
}
