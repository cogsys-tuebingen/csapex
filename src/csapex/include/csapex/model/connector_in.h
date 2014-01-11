#ifndef CONNECTOR_IN_H
#define CONNECTOR_IN_H

/// COMPONENT
#include <csapex/model/connectable.h>
#include <csapex/csapex_fwd.h>
#include <csapex/model/message.h>

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
    typename R::Ptr getMessage(typename boost::enable_if<boost::is_base_of<ConnectionType, R> >::type* dummy = 0) {
        QMutexLocker lock(&io_mutex);
        typename R::Ptr result = boost::dynamic_pointer_cast<R> (message_);
        assert(result || !message_);
        return result;
    }
    template <typename R>
    typename R::Ptr getMessage(typename boost::disable_if<boost::is_base_of<ConnectionType, R> >::type* dummy = 0) {
        QMutexLocker lock(&io_mutex);
        typename connection_types::GenericMessage<R>::Ptr tmp =
        boost::dynamic_pointer_cast<typename connection_types::GenericMessage<R> > (message_);
        return tmp->value;
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
