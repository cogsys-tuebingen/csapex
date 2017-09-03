/// HEADER
#include <csapex/model/connector.h>

/// PROJECT
#include <csapex/model/connectable_owner.h>
#include <csapex/model/connection.h>

/// SYSTEM
#include <sstream>

using namespace csapex;

Connector::Connector(const UUID &uuid, ConnectableOwnerWeakPtr owner)
    : Unique(uuid),
      owner_(owner)
{

}

bool Connector::isSynchronous() const
{
    return true;
}
bool Connector::isAsynchronous() const
{
    return !isSynchronous();
}

ConnectorDescription Connector::getDescription() const
{
    ConnectorDescription res(getOwner()->getUUID().getAbsoluteUUID(),
                             getUUID(),
                             getConnectorType(),
                             getType(),
                             getLabel(),
                             isOptional(),
                             isParameter());

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

std::string Connector::makeStatusString() const
{
    std::stringstream status_stream;
    status_stream << "UUID: " << getUUID();
    status_stream << ", Type: " << getType()->descriptiveName();
    status_stream << ", Connections: " << getConnections().size();
    status_stream << ", Messages: " << getCount();
    status_stream << ", Enabled: " << isEnabled();
    status_stream << ", #: " << sequenceNumber();

    addStatusInformation(status_stream);

    return status_stream.str();
}

void Connector::addStatusInformation(std::stringstream& status_stream) const
{
    // do nothing in base class
}
