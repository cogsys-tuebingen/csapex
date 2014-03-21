#ifndef CONNECTOR_OUT_H
#define CONNECTOR_OUT_H

/// COMPONENT
#include <csapex/model/connectable.h>
#include <csapex/csapex_fwd.h>
#include <csapex/model/message.h>
#include <csapex/utility/timable.h>

namespace csapex
{

class ConnectorOut : public virtual Connectable, public Timable
{
    friend class ConnectorIn;
    friend class ConnectorForward;
    friend class Graph;
    friend class command::AddConnection;
    friend class command::MoveConnection;
    friend class command::DeleteConnection;
    friend class DesignerIO;

    typedef std::vector<ConnectorIn*> TargetList;

public:
    typedef TargetList::const_iterator TargetIterator;

public:
    ConnectorOut(Settings& settings, const UUID &uuid);
    ConnectorOut(Settings& settings, Unique *parent, int sub_id);
    ~ConnectorOut();

    virtual bool canOutput() const {
        return true;
    }
    virtual bool isOutput() const {
        return true;
    }

    template <typename T>
    void publishIntegral(T message) {
        typename connection_types::DirectMessage<T>::Ptr msg(new connection_types::DirectMessage<T>);
        msg->value = message;
        publish(msg);
    }
    template <typename T>
    void publish(typename T::Ptr message,
                 typename boost::disable_if<boost::is_base_and_derived<connection_types::Message, T> >::type* dummy = 0) {
        typename connection_types::GenericMessage<T>::Ptr msg(new connection_types::GenericMessage<T>);
        msg->value = message;
        publish(msg);
    }
    template <class Container, typename T>
    void publish(const typename Container::template TypeMap<T>::Ptr& message,
                 typename boost::disable_if<boost::is_base_and_derived<connection_types::Message, T> >::type* dummy = 0) {
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

    void connectForcedWithoutCommand(ConnectorIn* other_side);

    virtual Command::Ptr removeAllConnectionsCmd();
    virtual Command::Ptr removeConnectionCmd(ConnectorIn *other_side);

    void forceSendMessage(bool force = true);
    ConnectionType::Ptr getMessage();

    virtual void updateIsProcessing();

protected:
    /// PRIVATE: Use command to create a connection (undoable)
    virtual bool tryConnect(Connectable* other_side);
    virtual void removeConnection(Connectable* other_side);
    virtual void removeAllConnectionsNotUndoable();

    bool connect(Connectable* other_side);

protected:
    std::vector<ConnectorIn*> targets_;
    bool force_send_message_;

    ConnectionType::Ptr message_;
};

}

#endif // CONNECTOR_OUT_H
