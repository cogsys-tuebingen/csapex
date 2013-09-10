/// HEADER
#include <csapex/command/delete_connector.h>

/// COMPONENT
#include <csapex/model/box.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex/model/graph.h>
#include <csapex/command/dispatcher.h>

using namespace csapex;
using namespace command;

DeleteConnector::DeleteConnector(Connector *_c) :
    in(_c->isInput()),
    c(_c)
{
    assert(c);
    c_uuid = c->UUID();
}

bool DeleteConnector::execute()
{
    Box::Ptr box_c = Graph::root()->findConnectorOwner(c_uuid);

    if(c->isConnected()) {
        if(in) {
//            delete_connections = ((ConnectorIn*) c)->removeAllConnectionsCmd();
            delete_connections = dynamic_cast<ConnectorIn*>(c)->removeAllConnectionsCmd();
        } else {
            delete_connections = dynamic_cast<ConnectorOut*>(c)->removeAllConnectionsCmd();
        }
        CommandDispatcher::execute(delete_connections);
    }

    if(in) {
        box_c->removeInput(dynamic_cast<ConnectorIn*>(c));
    } else {
        box_c->removeOutput(dynamic_cast<ConnectorOut*>(c));
    }

    return true;
}

bool DeleteConnector::undo()
{
    if(!refresh()) {
        return false;
    }

    return false;
}

bool DeleteConnector::redo()
{
    return false;
}

bool DeleteConnector::refresh()
{
    Box::Ptr box_c = Graph::root()->findConnectorOwner(c_uuid);

    if(!box_c) {
        return false;
    }

    if(in) {
        c = box_c->getInput(c_uuid);
    } else {
        c = box_c->getOutput(c_uuid);
    }

    assert(c);

    return true;
}


