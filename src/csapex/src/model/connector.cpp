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


ConnectableOwnerPtr Connector::getOwner() const
{
    return owner_.lock();
}

void Connector::addStatusInformation(std::stringstream& status_stream) const
{
    // do nothing in base class
}
