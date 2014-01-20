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
    ConnectorForward(bool primary_function_is_input, const UUID& uuid);
    ConnectorForward(Unique* parent, bool primary_function_is_input, int sub_id);
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


    virtual bool targetsCanBeMovedTo(Connectable* other_side) const;

    virtual void connectionMovePreview(Connectable* other_side);

    virtual Command::Ptr removeAllConnectionsCmd();

    void setPrimaryFunction(bool input);
    virtual void updateIsProcessing();

protected:
    virtual bool tryConnect(Connectable* other_side);
    virtual bool acknowledgeConnection(Connectable* other_side);
    virtual void removeConnection(Connectable* other_side);
    virtual void removeAllConnectionsNotUndoable();

private:
    bool primary_function_is_input;
};

}

#endif // CONNECTOR_FORWARD_H
