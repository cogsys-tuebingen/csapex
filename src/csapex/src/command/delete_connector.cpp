/// HEADER
#include <csapex/command/delete_connector.h>

/// COMPONENT

#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex/model/graph.h>
#include <csapex/model/node.h>
#include <csapex/command/dispatcher.h>

using namespace csapex;
using namespace command;

DeleteConnector::DeleteConnector(Connectable *_c) :
    in(_c->canInput()),
    c(_c)
{
    assert(c);
    c_uuid = c->getUUID();
}

std::string DeleteConnector::getType() const
{
    return "DeleteConnector";
}

std::string DeleteConnector::getDescription() const
{
    return std::string("deleted connector with UUID ") + c_uuid;
}


bool DeleteConnector::doExecute()
{
    Node* node = graph_->findNodeForConnector(c_uuid);

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
        node->removeInput(dynamic_cast<ConnectorIn*>(c));
    } else {
        node->removeOutput(dynamic_cast<ConnectorOut*>(c));
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
    Node* node = graph_->findNodeForConnector(c_uuid);

    if(!node) {
        return false;
    }

    if(in) {
        c = node->getInput(c_uuid);
    } else {
        c = node->getOutput(c_uuid);
    }

    assert(c);

    return true;
}


