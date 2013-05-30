#ifndef CONNECTOR_OUT_H
#define CONNECTOR_OUT_H

/// COMPONENT
#include "connector.h"

/// FORWARD DECLARATION
class ConnectorIn;

class ConnectorOut : public Connector
{
public:
    ConnectorOut(QWidget *parent);
    ~ConnectorOut();

    virtual bool tryConnect(Connector* other_side);
    virtual bool canConnect();
    virtual bool isConnected();
    virtual void removeConnection(Connector *other_side);

    virtual bool isOutput() {
        return true;
    }

protected:
    std::vector<ConnectorIn*> targets_;
};

#endif // CONNECTOR_OUT_H
