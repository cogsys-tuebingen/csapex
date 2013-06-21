#ifndef CONNECTOR_OUT_H
#define CONNECTOR_OUT_H

/// COMPONENT
#include "connector.h"

namespace vision_evaluator
{

/// FORWARD DECLARATION
class ConnectorIn;

class ConnectorOut : public Connector
{
    Q_OBJECT

    friend class ConnectorIn;
    friend class command::AddConnection;
    friend class DesignerIO;

    typedef std::vector<ConnectorIn*> TargetList;

public:
    typedef TargetList::iterator TargetIterator;

public:
    ConnectorOut(Box* parent, int sub_id);
    ~ConnectorOut();

    virtual bool isOutput() {
        return true;
    }

    virtual void publish(ConnectionType::Ptr message);

    virtual bool canConnect();
    virtual bool isConnected();

    TargetIterator beginTargets();
    TargetIterator endTargets();

private:
    /// PRIVATE: Use command to create a connection (undoable)
    virtual bool tryConnect(Connector* other_side);
    virtual void removeConnection(Connector* other_side);
    virtual void removeAllConnections();

protected:
    TargetList targets_;
};

}

#endif // CONNECTOR_OUT_H
