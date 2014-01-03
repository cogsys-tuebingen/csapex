#ifndef CONNECTOR_IN_H
#define CONNECTOR_IN_H

/// COMPONENT
#include <csapex/model/connectable.h>
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <QMutex>
#include <QWaitCondition>

namespace csapex
{

class ConnectorIn : public virtual Connectable
{
    Q_OBJECT

    friend class ConnectorOut;
    friend class ConnectorInForward;
    friend class command::AddConnection;
    friend class command::MoveConnection;
    friend class command::DeleteConnection;

public:
    ConnectorIn(const std::string& uuid);
    ConnectorIn(Unique *parent, int sub_id);
    virtual ~ConnectorIn();

    virtual bool canInput() const {
        return true;
    }
    virtual bool isInput() const {
        return true;
    }

    virtual bool canConnectTo(Connectable* other_side, bool move) const;

    void notify();
    void wait();
    void inputMessage(ConnectionType::Ptr message);

    bool hasMessage() const;
    template <typename R>
    typename R::Ptr getMessage() {
        QMutexLocker lock(&io_mutex);
        return boost::dynamic_pointer_cast<R> (message_);
    }

    virtual ConnectionType::Ptr getMessage()  __attribute__ ((deprecated));

    virtual bool targetsCanBeMovedTo(Connectable* other_side) const;
    virtual bool isConnected() const;

    virtual void connectionMovePreview(Connectable* other_side);
    virtual void validateConnections();

    Connectable* getSource() const;

    virtual Command::Ptr removeAllConnectionsCmd();

    bool isOptional() const;
    void setOptional(bool optional);

    bool isAsync() const;
    void setAsync(bool asynch);


protected:
    virtual bool tryConnect(Connectable* other_side);
    virtual bool acknowledgeConnection(Connectable* other_side);
    virtual void removeConnection(Connectable* other_side);
    virtual void removeAllConnectionsNotUndoable();

//public Q_SLOTS:
//    void relayMessage(ConnectorIn* source);

protected:
    Connectable* target;

    ConnectionType::Ptr message_;

    QMutex io_mutex;
    QMutex reserve_mutex;
    bool can_process;
    QWaitCondition can_process_cond;

    bool optional_;
    bool async_;
};

}

#endif // CONNECTOR_IN_H
