#ifndef CONNECTOR_IN_H
#define CONNECTOR_IN_H

/// COMPONENT
#include <csapex/model/connectable.h>
#include <csapex/csapex_fwd.h>
#include <csapex/model/message.h>
#include <csapex/utility/buffer.h>

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
    typename R::Ptr getMessage(typename boost::enable_if<boost::is_base_of<ConnectionType, R> >::type* /*dummy*/ = 0) {
        return buffer_->read<R>();
    }

    template <typename R>
    typename R::Ptr getMessage(typename boost::disable_if<boost::is_base_of<ConnectionType, R> >::type* /*dummy*/ = 0) {
        return buffer_->read< connection_types::GenericMessage<R> >() -> value;
    }

    template <typename Container, typename R>
    boost::shared_ptr<typename Container::template TypeMap<R>::type const>
    getMessage() {
        return buffer_->read<Container>() -> template makeShared<R>();
    }

    virtual bool targetsCanBeMovedTo(Connectable* other_side) const;
    virtual bool isConnected() const;

    virtual void connectionMovePreview(Connectable* other_side);
    virtual void validateConnections();

    Connectable* getSource() const;

    virtual Command::Ptr removeAllConnectionsCmd();

    bool isOptional() const;
    void setOptional(bool optional);

    bool hasMessage() const;
    void free();

    virtual void disable();

protected:
    virtual bool tryConnect(Connectable* other_side);
    virtual bool acknowledgeConnection(Connectable* other_side);
    virtual void removeConnection(Connectable* other_side);
    virtual void removeAllConnectionsNotUndoable();

protected:
    Connectable* target;

    BufferPtr buffer_;

    bool optional_;
};

}

#endif // CONNECTOR_IN_H
