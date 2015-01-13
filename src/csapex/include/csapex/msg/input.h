#ifndef CONNECTOR_IN_H
#define CONNECTOR_IN_H

/// COMPONENT
#include <csapex/model/connectable.h>
#include <csapex/csapex_fwd.h>
#include <csapex/msg/message.h>
#include <csapex/utility/buffer.h>

namespace csapex
{

class Input : public Connectable
{
    Q_OBJECT

    friend class Output;
    friend class command::AddConnection;
    friend class command::MoveConnection;
    friend class command::DeleteConnection;

public:
    Input(const UUID &uuid);
    Input(Unique *parent, int sub_id);
    virtual ~Input();

    virtual bool canInput() const {
        return true;
    }
    virtual bool isInput() const {
        return true;
    }

    virtual bool canConnectTo(Connectable* other_side, bool move) const;

    void inputMessage(ConnectionType::ConstPtr message);

    template <typename R>
    std::shared_ptr<R const>
    getMessage(typename boost::enable_if<boost::is_base_of<ConnectionType, R> >::type* /*dummy*/ = 0) const
    {
        return buffer_->read<R>();
    }

    template <typename R>
    std::shared_ptr<R const>
    getMessage(typename boost::disable_if<boost::is_base_of<ConnectionType, R> >::type* /*dummy*/ = 0) const
    {
        return buffer_->read< connection_types::GenericPointerMessage<R> >() -> value;
    }

    template <typename Container, typename R>
    std::shared_ptr<typename Container::template TypeMap<R>::type const>
    getMessage() const
    {
        auto msg = buffer_->read<Container>();
        return msg -> template makeShared<R>();
    }


    template <typename R>
    std::shared_ptr<R>
    getClonedMessage() const
    {
        const auto& msg = getMessage<R>();
        if(msg == nullptr) {
            return nullptr;
        }
        return std::dynamic_pointer_cast<R>(msg->clone());
    }


    template <typename R>
    R getValue() const
    {
        const auto& msg = getMessage< connection_types::GenericValueMessage<R> >();
        if(!msg) {
            throw std::logic_error("cannot convert message to requested value");
        }
        return msg->value;
    }

    template <typename R>
    bool isValue() {
        return isMessage< connection_types::GenericValueMessage<R> >();
    }

    virtual bool targetsCanBeMovedTo(Connectable* other_side) const;
    virtual bool isConnected() const;

    virtual void connectionMovePreview(Connectable* other_side);
    virtual void validateConnections();

    Connectable* getSource() const;

    virtual CommandPtr removeAllConnectionsCmd();

    bool isOptional() const;
    void setOptional(bool optional);

    template <typename T>
    bool isMessage() {
        std::lock_guard<std::recursive_mutex> lock(sync_mutex);
        return buffer_->isType<T>();
    }

    bool hasMessage() const;
    bool hasReceived() const;

    void free();
    void stop();

    virtual void enable();
    virtual void disable();

    virtual void notifyMessageProcessed();

    void reset();

protected:
    virtual bool tryConnect(Connectable* other_side);
    virtual bool acknowledgeConnection(Connectable* other_side);
    virtual void removeConnection(Connectable* other_side);
    virtual void removeAllConnectionsNotUndoable();

Q_SIGNALS:
    void gotMessage(ConnectionType::ConstPtr msg);

private Q_SLOTS:
    void handleMessage(ConnectionType::ConstPtr msg);

protected:
    Connectable* target;

    BufferPtr buffer_;

    bool optional_;
};

}

#endif // CONNECTOR_IN_H
