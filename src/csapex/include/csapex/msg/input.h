#ifndef CONNECTOR_IN_H
#define CONNECTOR_IN_H

/// COMPONENT
#include <csapex/model/connectable.h>
#include <csapex/csapex_fwd.h>
#include <csapex/msg/message.h>
#include <csapex/msg/generic_pointer_message.hpp>
#include <csapex/utility/buffer.h>

/// SYSTEM
#include <QMutex>
#include <QWaitCondition>

namespace csapex
{

class Input : public virtual Connectable
{
    Q_OBJECT

    friend class Output;
    friend class InputForward;
    friend class command::AddConnection;
    friend class command::MoveConnection;
    friend class command::DeleteConnection;

public:
    Input(Settings &settings, const UUID &uuid);
    Input(Settings& settings, Unique *parent, int sub_id);
    virtual ~Input();

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
        return buffer_->read< connection_types::GenericPointerMessage<R> >() -> value;
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

    virtual CommandPtr removeAllConnectionsCmd();

    bool isOptional() const;
    void setOptional(bool optional);

    virtual void setAsync(bool asynch);

    template <typename T>
    bool isMessage() {
        return buffer_->isType<T>();
    }

    bool hasMessage() const;
    void free();

    virtual void enable();
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
