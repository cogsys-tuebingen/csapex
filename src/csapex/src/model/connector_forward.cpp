/// HEADER
#include <csapex/model/connector_forward.h>

using namespace csapex;

ConnectorForward::ConnectorForward(Box* parent, bool primary_function_is_input, const std::string &uuid)
    : Connector(parent, uuid), ConnectorIn(parent, uuid), ConnectorOut(parent, uuid), primary_function_is_input(primary_function_is_input)
{

}

ConnectorForward::ConnectorForward(Box* parent, bool primary_function_is_input, int sub_id)
    : Connector(parent, sub_id, TYPE_MISC), ConnectorIn(parent, sub_id), ConnectorOut(parent, sub_id), primary_function_is_input(primary_function_is_input)
{

}

ConnectorForward::~ConnectorForward()
{

}

bool ConnectorForward::isForwarding() const
{
    return true;
}

bool ConnectorForward::isOutput() const
{
    return !primary_function_is_input;
}
bool ConnectorForward::isInput() const
{
    return primary_function_is_input;
}

void ConnectorForward::inputMessage(ConnectionType::Ptr message)
{
    publish(message);
}

bool ConnectorForward::tryConnect(Connector* other_side)
{
    bool use_in = false;

    if(other_side->isForwarding()) {
        if(isOutput() == other_side->isOutput()) {
            return false;
        }

        use_in = other_side->isOutput();

    } else {
        // other side is normal connector
        use_in = other_side->canOutput();
    }

    if(use_in) {
        // connection from "left"
        return ConnectorIn::tryConnect(other_side);
    } else {
        // connection from "right"
        return ConnectorOut::tryConnect(other_side);
    }
}

bool ConnectorForward::acknowledgeConnection(Connector* other_side)
{
    return ConnectorIn::acknowledgeConnection(other_side);
}

void ConnectorForward::removeConnection(Connector* other_side)
{    bool use_in = false;

     if(other_side->isForwarding()) {
         if(isOutput() == other_side->isOutput()) {
             return;
         }

         use_in = other_side->isOutput();

     } else {
         // other side is normal connector
         use_in = other_side->canOutput();
     }

     if(use_in) {
        ConnectorIn::removeConnection(other_side);
    } else {
        ConnectorOut::removeConnection(other_side);
    }
}

void ConnectorForward::removeAllConnectionsNotUndoable()
{
    if(primary_function_is_input) {
        ConnectorIn::removeAllConnectionsNotUndoable();
    } else {
        ConnectorOut::removeAllConnectionsNotUndoable();
    }
}

bool ConnectorForward::canConnect() const
{
    if(primary_function_is_input) {
        return ConnectorIn::canConnect();
    } else {
        return ConnectorOut::canConnect();
    }
}

bool ConnectorForward::isConnected() const
{
    if(primary_function_is_input) {
        return ConnectorIn::isConnected();
    } else {
        return ConnectorOut::isConnected();
    }
}

void ConnectorForward::validateConnections()
{
    if(primary_function_is_input) {
        ConnectorIn::validateConnections();
    } else {
        ConnectorOut::validateConnections();
    }
}

bool ConnectorForward::targetsCanConnectTo(Connector *other_side) const
{
    if(primary_function_is_input) {
        return ConnectorIn::targetsCanConnectTo(other_side);
    } else {
        return ConnectorOut::targetsCanConnectTo(other_side);
    }
}

void ConnectorForward::connectionMovePreview(Connector *other_side)
{
    if(primary_function_is_input) {
        return ConnectorIn::connectionMovePreview(other_side);
    } else {
        return ConnectorOut::connectionMovePreview(other_side);
    }
}

Command::Ptr ConnectorForward::removeAllConnectionsCmd()
{
    if(primary_function_is_input) {
        return  ConnectorIn::removeAllConnectionsCmd();
    } else {
        return ConnectorOut::removeAllConnectionsCmd();
    }
}

void ConnectorForward::setPrimaryFunction(bool input)
{
    primary_function_is_input = input;
}
