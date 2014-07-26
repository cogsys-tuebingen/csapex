#ifndef CONNECTOR_OUT_H
#define CONNECTOR_OUT_H

/// COMPONENT
#include <csapex/model/connectable.h>
#include <csapex/csapex_fwd.h>
#include <csapex/msg/generic_pointer_message.hpp>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/utility/timable.h>

namespace csapex
{

class Output : public virtual Connectable, public Timable
{
    friend class Input;
    friend class ConnectorForward;
    friend class Graph;
    friend class command::AddConnection;
    friend class command::MoveConnection;
    friend class command::DeleteConnection;
    friend class DesignerIO;

    typedef std::vector<Input*> TargetList;

public:
    typedef TargetList::const_iterator TargetIterator;

public:
    Output(Settings& settings, const UUID &uuid);
    Output(Settings& settings, Unique *parent, int sub_id);
    ~Output();

    virtual bool canOutput() const {
        return true;
    }
    virtual bool isOutput() const {
        return true;
    }

    template <typename T>
    void publishIntegral(T message, const std::string& frame_id = "/") {
        typename connection_types::GenericValueMessage<T>::Ptr msg(new connection_types::GenericValueMessage<T>(frame_id));
        msg->value = message;
        publish(msg);
    }
    template <typename T>
    void publish(typename T::Ptr message,
                 typename boost::disable_if<boost::is_base_and_derived<connection_types::Message, T> >::type* dummy = 0) {
        typename connection_types::GenericPointerMessage<T>::Ptr msg(new connection_types::GenericPointerMessage<T>);
        msg->value = message;
        publish(msg);
    }
    template <class Container, typename T>
    void publish(const typename Container::template TypeMap<T>::Ptr& message/*,
                 typename boost::disable_if<boost::is_base_and_derived<connection_types::Message, T> >::type* dummy = 0*/) {
        typename Container::Ptr msg(Container::template make<T>());
        msg->template set<T>(message);
        publish(msg);
    }

    void publish(ConnectionType::Ptr message);

    virtual bool targetsCanBeMovedTo(Connectable *other_side) const;
    virtual bool isConnected() const;

    virtual void connectionMovePreview(Connectable* other_side);
    virtual void validateConnections();

    int noTargets();
    TargetIterator beginTargets() const;
    TargetIterator endTargets() const;

    void connectForcedWithoutCommand(Input* other_side);

    virtual CommandPtr removeAllConnectionsCmd();
    virtual CommandPtr removeConnectionCmd(Input *other_side);

    void forceSendMessage(bool force = true);

    bool hasMessage();
    ConnectionType::Ptr getMessage();

    void sendMessages();

protected:
    /// PRIVATE: Use command to create a connection (undoable)
    virtual bool tryConnect(Connectable* other_side);
    virtual void removeConnection(Connectable* other_side);
    virtual void removeAllConnectionsNotUndoable();

    bool connect(Connectable* other_side);

protected:
    std::vector<Input*> targets_;
    bool force_send_message_;

    ConnectionType::Ptr message_;
    ConnectionType::Ptr message_to_send_;
};

}

#endif // CONNECTOR_OUT_H
