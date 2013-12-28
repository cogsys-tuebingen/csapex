#ifndef CONNECTOR_OUT_H
#define CONNECTOR_OUT_H

/// COMPONENT
#include "connector.h"
#include <csapex/csapex_fwd.h>

namespace csapex
{

class ConnectorOut : public virtual Connectable
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
    ConnectorOut(Node* parent, const std::string& uuid);
    ConnectorOut(Node* parent, int sub_id);
    ~ConnectorOut();

    virtual bool canOutput() const {
        return true;
    }
    virtual bool isOutput() const {
        return true;
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
