/// HEADER
#include <csapex/command/delete_connector.h>

/// COMPONENT
#include <csapex/model/node_handle.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/model/graph.h>
#include <csapex/model/node.h>
#include <csapex/command/dispatcher.h>
#include <csapex/command/command_factory.h>

using namespace csapex;
using namespace command;

DeleteConnector::DeleteConnector(const AUUID& parent_uuid, Connectable *_c)
    : CommandImplementation(parent_uuid), in(_c->canInput()), c_uuid(_c->getUUID())
{
}

std::string DeleteConnector::getDescription() const
{
    return std::string("deleted connector with UUID ") + c_uuid.getFullName();
}


bool DeleteConnector::doExecute()
{
    NodeHandle* node_handle = getGraph()->findNodeHandleForConnector(c_uuid);

    if(!node_handle) {
        return false;
    }

    ConnectablePtr c;
    if(in) {
        c = node_handle->getInput(c_uuid);
    } else {
        c = node_handle->getOutput(c_uuid);
    }

    apex_assert_hard(c);
    if(c->isConnected()) {
        delete_connections = CommandFactory(getRoot(), graph_uuid).removeAllConnectionsCmd(c);

        Command::executeCommand(delete_connections);
    }

    if(in) {
        node_handle->removeInput(c->getUUID());
    } else {
        node_handle->removeOutput(c->getUUID());
    }

    return true;
}

bool DeleteConnector::doUndo()
{
    return false;
}

bool DeleteConnector::doRedo()
{
    return false;
}


