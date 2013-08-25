/// HEADER
#include <csapex/connector_forward.h>

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

void ConnectorForward::inputMessage(ConnectionType::Ptr message)
{
    publish(message);
}

bool ConnectorForward::tryConnect(Connector* other_side)
{
    if(other_side->isOutput()) {
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
{
    if(other_side->isOutput()) {
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

bool ConnectorForward::canConnect()
{
    if(primary_function_is_input) {
        return ConnectorIn::canConnect();
    } else {
        return ConnectorOut::canConnect();
    }
}

bool ConnectorForward::isConnected()
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

bool ConnectorForward::targetsCanConnectTo(Connector *other_side)
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


bool ConnectorForward::shouldCreate(bool left, bool)
{
    bool full_input = primary_function_is_input && isConnected();
    return left && !full_input;
}

bool ConnectorForward::shouldMove(bool left, bool right)
{
    bool full_input = primary_function_is_input && isConnected();
    return (right && isConnected()) || (left && full_input);
}
