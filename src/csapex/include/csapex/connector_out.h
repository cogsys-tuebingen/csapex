#ifndef CONNECTOR_OUT_H
#define CONNECTOR_OUT_H

/// COMPONENT
#include "connector.h"

namespace csapex
{

/// FORWARD DECLARATION
class ConnectorIn;

namespace command {
class AddConnection;
class MoveConnection;
class DeleteConnection;
}

class ConnectorOut : public virtual Connector
{
    friend class ConnectorIn;
    friend class Graph;
    friend class command::AddConnection;
    friend class command::MoveConnection;
    friend class command::DeleteConnection;
    friend class DesignerIO;

    typedef std::vector<ConnectorIn*> TargetList;

public:
    typedef TargetList::iterator TargetIterator;

public:
    ConnectorOut(Box* parent, const std::string& uuid);
    ConnectorOut(Box* parent, int sub_id);
    ~ConnectorOut();

    virtual bool isOutput() const {
        return true;
    }

    virtual void publish(ConnectionType::Ptr message);

    virtual bool canConnect();
    virtual bool targetsCanConnectTo(Connector *other_side);
    virtual bool isConnected();

    virtual void connectionMovePreview(Connector* other_side);
    virtual void validateConnections();

    int noTargets();
    TargetIterator beginTargets();
    TargetIterator endTargets();

    void connectForcedWithoutCommand(ConnectorIn* other_side);

    virtual Command::Ptr removeAllConnectionsCmd();
    virtual Command::Ptr removeConnectionCmd(ConnectorIn *other_side);

    void forceSendMessage(bool force = true);
    ConnectionType::Ptr getMessage();


protected:
    /// PRIVATE: Use command to create a connection (undoable)
    virtual bool tryConnect(Connector* other_side);
    virtual void removeConnection(Connector* other_side);
    virtual void removeAllConnectionsNotUndoable();

protected:
    std::vector<ConnectorIn*> targets_;
    bool force_send_message_;

    ConnectionType::Ptr message_;
};

}

#endif // CONNECTOR_OUT_H
