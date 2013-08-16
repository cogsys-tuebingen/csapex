#ifndef CONNECTOR_IN_FORWARD_H
#define CONNECTOR_IN_FORWARD_H

/// COMPONENT
#include "connector_in.h"

namespace csapex
{

class ConnectorInForward : public ConnectorIn
{
public:
    ConnectorInForward(Box* parent, const std::string& uuid);
    ConnectorInForward(Box* parent, int sub_id);

    virtual ~ConnectorInForward();

    virtual bool canConnect();
    virtual bool isConnected();

    virtual void validateConnections();

    virtual bool isForwarding() const;
    virtual void inputMessage(ConnectionType::Ptr message);

protected:
    virtual bool tryConnect(Connector* other_side);
    virtual bool acknowledgeConnection(Connector* other_side);
    virtual void removeConnection(Connector* other_side);
    virtual void removeAllConnectionsNotUndoable();
};

}


#endif // CONNECTOR_IN_FORWARD_H
