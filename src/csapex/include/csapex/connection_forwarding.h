#ifndef CONNECTION_FORWARDING_H
#define CONNECTION_FORWARDING_H

/// COMPONENT
#include <csapex/connection.h>

namespace csapex
{
class ConnectorInForward;
class ConnectorOutForward;

class ConnectionForwarding : public Connection
{
public:
    ConnectionForwarding(ConnectorOut* from, ConnectorOutForward* to);
    ConnectionForwarding(ConnectorInForward* from, ConnectorIn* to);

private:
    bool in;
};

}

#endif // CONNECTION_FORWARDING_H
