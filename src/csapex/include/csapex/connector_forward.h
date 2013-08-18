#ifndef CONNECTOR_FORWARD_H
#define CONNECTOR_FORWARD_H

/// COMPONENT
#include <csapex/connector_in.h>
#include <csapex/connector_out.h>

namespace csapex
{

class ConnectorForward : public ConnectorIn, public ConnectorOut
{
    Q_OBJECT

public:
    ConnectorForward(Box* parent, bool primary_function_is_input, const std::string& uuid);
    ConnectorForward(Box* parent, bool primary_function_is_input, int sub_id);
    virtual ~ConnectorForward();

    virtual bool isInput() const {
        return true;
    }
    virtual bool isOutput() const {
        return true;
    }
    virtual bool isForwarding() const;


    virtual void inputMessage(ConnectionType::Ptr message);


    virtual bool canConnect();
    virtual bool isConnected();

    virtual void validateConnections();


    virtual bool targetsCanConnectTo(Connector* other_side);

    virtual void connectionMovePreview(Connector* other_side);

    virtual Command::Ptr removeAllConnectionsCmd();

    void setPrimaryFunction(bool input);

protected:
    virtual bool tryConnect(Connector* other_side);
    virtual bool acknowledgeConnection(Connector* other_side);
    virtual void removeConnection(Connector* other_side);
    virtual void removeAllConnectionsNotUndoable();

    virtual bool shouldMove(bool left, bool right);
    virtual bool shouldCreate(bool left, bool right);

private:
    bool primary_function_is_input;
};

}

#endif // CONNECTOR_FORWARD_H
