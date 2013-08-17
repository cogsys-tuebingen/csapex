/// HEADER
#include <csapex/connection_forwarding.h>

/// COMPONENT
#include <csapex/connector_in.h>
#include <csapex/connector_out.h>
#include <csapex/connector_in_forward.h>
#include <csapex/connector_out_forward.h>

using namespace csapex;

ConnectionForwarding::ConnectionForwarding(ConnectorOut *from, ConnectorOutForward *to)
    : Connection(from, to), in(false)
{
    QObject::connect(from, SIGNAL(messageSent(ConnectorOut*)), to, SLOT(relayMessage(ConnectorOut*)));
    from->forceSendMessage();
}

ConnectionForwarding::ConnectionForwarding(ConnectorInForward* from, ConnectorIn* to)
    : Connection(from, to), in(true)
{
    QObject::connect(from, SIGNAL(messageArrived(ConnectorIn*)), to, SLOT(relayMessage(ConnectorIn*)));
}
