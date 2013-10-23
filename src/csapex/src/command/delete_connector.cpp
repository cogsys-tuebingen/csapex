/// HEADER
#include <csapex/command/delete_connector.h>

/// COMPONENT
#include <csapex/model/box.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex/model/graph.h>
#include <csapex/model/node.h>
#include <csapex/command/dispatcher.h>

using namespace csapex;
using namespace command;

DeleteConnector::DeleteConnector(Connector *_c) :
    in(_c->canInput()),
    c(_c)
{
    assert(c);
    c_uuid = c->UUID();
}

bool DeleteConnector::doExecute()
{
    Box::Ptr box_c = graph_->findConnectorOwner(c_uuid);

    if(c->isConnected()) {
        if(in) {
//            delete_connections = ((ConnectorIn*) c)->removeAllConnectionsCmd();
            delete_connections = dynamic_cast<ConnectorIn*>(c)->removeAllConnectionsCmd();
        } else {
            delete_connections = dynamic_cast<ConnectorOut*>(c)->removeAllConnectionsCmd();
        }
        Command::executeCommand(graph_, delete_connections);
    }

    if(in) {
        box_c->getNode()->removeInput(dynamic_cast<ConnectorIn*>(c));
    } else {
        box_c->getNode()->removeOutput(dynamic_cast<ConnectorOut*>(c));
    }

    return true;
}

bool DeleteConnector::doUndo()
{
    if(!refresh()) {
        return false;
    }

    return false;
}

bool DeleteConnector::doRedo()
{
    return false;
}

bool DeleteConnector::refresh()
{
    Box::Ptr box_c = graph_->findConnectorOwner(c_uuid);

    if(!box_c) {
        return false;
    }

    if(in) {
        c = box_c->getNode()->getInput(c_uuid);
    } else {
        c = box_c->getNode()->getOutput(c_uuid);
    }

    assert(c);

    return true;
}


