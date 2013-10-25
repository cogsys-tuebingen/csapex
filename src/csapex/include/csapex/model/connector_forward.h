#ifndef CONNECTOR_FORWARD_H
#define CONNECTOR_FORWARD_H

/// COMPONENT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>

namespace csapex
{

class ConnectorForward : public ConnectorIn, public ConnectorOut
{
    Q_OBJECT

public:
    ConnectorForward(Node* parent, bool primary_function_is_input, const std::string& uuid);
    ConnectorForward(Node* parent, bool primary_function_is_input, int sub_id);
    virtual ~ConnectorForward();

    virtual bool canInput() const {
        return true;
    }
    virtual bool canOutput() const {
        return true;
    }
    virtual bool isForwarding() const;

    virtual bool isOutput() const;
    virtual bool isInput() const;


    virtual void inputMessage(ConnectionType::Ptr message);


    virtual bool isConnected() const;

    virtual void validateConnections();


    virtual bool targetsCanBeMovedTo(Connector* other_side) const;

    virtual void connectionMovePreview(Connector* other_side);

    virtual Command::Ptr removeAllConnectionsCmd();

    void setPrimaryFunction(bool input);

protected:
    virtual bool tryConnect(Connector* other_side);
    virtual bool acknowledgeConnection(Connector* other_side);
    virtual void removeConnection(Connector* other_side);
    virtual void removeAllConnectionsNotUndoable();

private:
    bool primary_function_is_input;
};

}

#endif // CONNECTOR_FORWARD_H
