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

DeleteConnector::DeleteConnector(Connectable *_c)
    : in(_c->canInput()), c(_c), c_uuid(c->getUUID())
{
}

std::string DeleteConnector::getType() const
{
    return "DeleteConnector";
}

std::string DeleteConnector::getDescription() const
{
    return std::string("deleted connector with UUID ") + c_uuid.getFullName();
}


bool DeleteConnector::doExecute()
{
    NodeHandle* node_worker = graph_->findNodeHandleForConnector(c_uuid);

    if(c->isConnected()) {
        delete_connections = CommandFactory(graph_).removeAllConnectionsCmd(c);

        Command::executeCommand(graph_worker_, graph_, thread_pool_, node_factory_, delete_connections);
    }

    if(in) {
        node_worker->removeInput(c->getUUID());
    } else {
        node_worker->removeOutput(c->getUUID());
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
    NodeHandle* node_handle = graph_->findNodeHandleForConnector(c_uuid);

    if(!node_handle) {
        return false;
    }

    if(in) {
        c = node_handle->getInput(c_uuid);
    } else {
        c = node_handle->getOutput(c_uuid);
    }

    apex_assert_hard(c);

    return true;
}


