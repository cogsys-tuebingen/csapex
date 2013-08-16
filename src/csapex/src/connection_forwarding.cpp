/// HEADER
#include <csapex/connection_forwarding.h>

/// COMPONENT
#include <csapex/connector_in.h>
#include <csapex/connector_out.h>
#include <csapex/connector_in_forward.h>
#include <csapex/connector_out_forward.h>

using namespace csapex;

ConnectionForwarding::ConnectionForwarding(ConnectorOutForward* from, ConnectorOut* to)
    : Connection(from, to), in(false)
{
    QObject::connect(from, SIGNAL(messageSent(ConnectorOut*)), to, SIGNAL(relayMessage(ConnectorOut*)));
}

ConnectionForwarding::ConnectionForwarding(ConnectorInForward* from, ConnectorIn* to)
    : Connection(from, to), in(true)
{
    QObject::connect(from, SIGNAL(messageArrived(ConnectorIn*)), to, SLOT(relayMessage(ConnectorIn*)));
}
