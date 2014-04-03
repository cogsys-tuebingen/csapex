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
    ConnectorIn(Settings &settings, const UUID &uuid);
    ConnectorIn(Settings& settings, Unique *parent, int sub_id);
    virtual ~ConnectorIn();

    virtual bool canInput() const {
        return true;
    }
    virtual bool isInput() const {
        return true;
    }

    virtual bool canConnectTo(Connectable* other_side, bool move) const;

    void inputMessage(ConnectionType::Ptr message);

    template <typename R>
    typename R::Ptr getMessage(typename boost::enable_if<boost::is_base_of<ConnectionType, R> >::type* dummy = 0) {
        waitForMessage();

        QMutexLocker lock(&io_mutex);

        if(!message_) {
            std::stringstream e;
            e << "message in connector " << getUUID().getFullName() << " is empty!";
            throw std::runtime_error(e.str());
        }

        typename R::Ptr result = boost::dynamic_pointer_cast<R> (message_);
        if(result) {
            return result;
        } else {
            std::stringstream e;
            e << "cannot cast message from " << message_->toType()->name() << " to " << type2name(typeid(R));
            throw std::runtime_error(e.str());
        }
    }
    template <typename R>
    typename R::Ptr getMessage(typename boost::disable_if<boost::is_base_of<ConnectionType, R> >::type* dummy = 0) {
        waitForMessage();

        QMutexLocker lock(&io_mutex);
        typename connection_types::GenericMessage<R>::Ptr tmp =
        boost::dynamic_pointer_cast<typename connection_types::GenericMessage<R> > (message_);
        return tmp->value;
    }
    template <typename Container, typename R>
    boost::shared_ptr<typename Container::template TypeMap<R>::type const>
    getMessage() {
        waitForMessage();

        QMutexLocker lock(&io_mutex);
        typename Container::Ptr tmp = boost::dynamic_pointer_cast<Container>(message_);
        if(!tmp) {
            throw std::runtime_error(std::string("cannot get message, not of type ") + type2name(typeid(Container)));
        }
        return tmp->template makeShared<R>();
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

    bool isLegacy() const;
    void setLegacy(bool legacy);

    bool hasMessage() const;

    virtual void waitForProcessing();
    virtual void setProcessing(bool processing);
    virtual void updateIsProcessing();

protected:
    virtual bool tryConnect(Connectable* other_side);
    virtual bool acknowledgeConnection(Connectable* other_side);
    virtual void removeConnection(Connectable* other_side);
    virtual void removeAllConnectionsNotUndoable();

private:
    void waitForMessage();

protected:
    Connectable* target;

    ConnectionType::Ptr message_;

    bool optional_;
    bool legacy_;
};

}

#endif // CONNECTOR_IN_H
