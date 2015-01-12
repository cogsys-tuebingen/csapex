#ifndef CONNECTOR_OUT_H
#define CONNECTOR_OUT_H

/// COMPONENT
#include <csapex/model/connectable.h>
#include <csapex/msg/message_traits.h>
#include <csapex/csapex_fwd.h>

namespace csapex
{

class Output : public Connectable
{
    friend class Input;
    friend class Graph;
    friend class command::AddConnection;
    friend class command::MoveConnection;
    friend class command::DeleteConnection;
    friend class DesignerIO;

public:
    Output(const UUID &uuid);
    Output(Unique *parent, int sub_id);
    ~Output();

    virtual bool canOutput() const override
    {
        return true;
    }
    virtual bool isOutput() const override
    {
        return true;
    }

    virtual void disable() override;


    template <typename T>
    void publish(typename T::Ptr message,
                 std::string frame_id = "/",
                 typename boost::enable_if<connection_types::should_use_pointer_message<T> >::type* = 0) {
        typename connection_types::GenericPointerMessage<T>::Ptr msg(new connection_types::GenericPointerMessage<T>(frame_id));
        msg->value = message;
        publish(boost::dynamic_pointer_cast<ConnectionType>(msg));
    }

    template <typename T>
    void publish(T message,
                 std::string frame_id = "/",
                 typename boost::enable_if<connection_types::should_use_value_message<T> >::type* = 0) {
        typename connection_types::GenericValueMessage<T>::Ptr msg(new connection_types::GenericValueMessage<T>(frame_id));
        msg->value = message;
        publish(boost::dynamic_pointer_cast<ConnectionType>(msg));
    }

    template <class Container, typename T>
    void publish(const typename Container::template TypeMap<T>::Ptr& message) {
        typename Container::Ptr msg(Container::template make<T>());
        msg->template set<T>(message);
        publish(msg);
    }

    void publish(ConnectionType::ConstPtr message);

    virtual bool targetsCanBeMovedTo(Connectable *other_side) const override;
    virtual bool isConnected() const override;

    virtual void connectionMovePreview(Connectable* other_side) override;
    virtual void validateConnections() override;

    int noTargets();
    std::vector<Input*> getTargets() const;

    void connectForcedWithoutCommand(Input* other_side);

    virtual CommandPtr removeAllConnectionsCmd() override;
    virtual CommandPtr removeConnectionCmd(Input *other_side);

    void forceSendMessage(bool force = true);

    bool hasMessage();
    ConnectionType::ConstPtr getMessage();

    bool canSendMessages();
    void sendMessages();

    void reset();
    void clearMessage();

protected:
    /// PRIVATE: Use command to create a connection (undoable)
    virtual bool tryConnect(Connectable* other_side) override;
    virtual void removeConnection(Connectable* other_side) override;
    virtual void removeAllConnectionsNotUndoable() override;

    bool connect(Connectable* other_side);

protected:
    std::vector<Input*> targets_;
    bool force_send_message_;

    ConnectionType::ConstPtr message_;
    ConnectionType::ConstPtr message_to_send_;
};

}

#endif // CONNECTOR_OUT_H
