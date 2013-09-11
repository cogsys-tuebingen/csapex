#ifndef CONNECTOR_IN_H
#define CONNECTOR_IN_H

/// COMPONENT
#include "connector.h"
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <QMutex>

namespace csapex
{

class ConnectorIn : public virtual Connector
{
    Q_OBJECT

    friend class ConnectorOut;
    friend class ConnectorInForward;
    friend class command::AddConnection;
    friend class command::MoveConnection;
    friend class command::DeleteConnection;

public:
    ConnectorIn(Box* parent, const std::string& uuid);
    ConnectorIn(Box* parent, int sub_id);
    virtual ~ConnectorIn();

    virtual bool canInput() const {
        return true;
    }
    virtual bool isInput() const {
        return true;
    }

    virtual void inputMessage(ConnectionType::Ptr message);
    virtual ConnectionType::Ptr getMessage();

    virtual bool canConnect() const;
    virtual bool targetsCanConnectTo(Connector* other_side) const;
    virtual bool isConnected() const;

    virtual void connectionMovePreview(Connector* other_side);
    virtual void validateConnections();

    Connector* getConnected() const;

    virtual Command::Ptr removeAllConnectionsCmd();

protected:
    virtual bool tryConnect(Connector* other_side);
    virtual bool acknowledgeConnection(Connector* other_side);
    virtual void removeConnection(Connector* other_side);
    virtual void removeAllConnectionsNotUndoable();

//public Q_SLOTS:
//    void relayMessage(ConnectorIn* source);

protected:
    Connector* target;

    ConnectionType::Ptr message_;

    QMutex io_mutex;
};

}

#endif // CONNECTOR_IN_H
