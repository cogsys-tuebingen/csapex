#ifndef CONNECTOR_IN_H
#define CONNECTOR_IN_H

/// COMPONENT
#include "connector.h"
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <QMutex>
#include <QWaitCondition>

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

    virtual bool canConnectTo(Connector* other_side, bool move) const;

    void notify();
    void wait();
    void inputMessage(ConnectionType::Ptr message);

    template <typename R>
    typename R::Ptr getMessage() {
        return boost::dynamic_pointer_cast<R> (getMessage());
    }

    virtual ConnectionType::Ptr getMessage();

    virtual bool targetsCanBeMovedTo(Connector* other_side) const;
    virtual bool isConnected() const;

    virtual void connectionMovePreview(Connector* other_side);
    virtual void validateConnections();

    Connector* getConnected() const;

    virtual Command::Ptr removeAllConnectionsCmd();

    bool isOptional() const;
    void setOptional(bool optional);

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
    QMutex reserve_mutex;
    bool can_process;
    QWaitCondition can_process_cond;

    bool optional_;
};

}

#endif // CONNECTOR_IN_H
