/// HEADER
#include <csapex/command/delete_node.h>

/// COMPONENT
#include <csapex/command/delete_connection.h>
#include <csapex/command/command_factory.h>
#include <csapex/model/node_constructor.h>
#include <csapex/model/node.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_worker.h>
#include <csapex/model/node_state.h>
#include <csapex/factory/node_factory.h>
#include <csapex/model/graph.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>

/// SYSTEM

using namespace csapex::command;

DeleteNode::DeleteNode(const UUID& uuid)
    : Meta("delete node and connections"), uuid(uuid)
{
}

std::string DeleteNode::getType() const
{
    return "DeleteNode";
}

std::string DeleteNode::getDescription() const
{
    return std::string("deleted node ") + uuid.getFullName();
}


bool DeleteNode::doExecute()
{
    NodeHandle* node_handle = graph_->findNodeHandleForConnector(uuid);

    type = node_handle->getType();

    locked = false;
    clear();

    for(auto connectable : node_handle->getAllConnectors()) {
        if(connectable->isConnected()) {
            add(CommandFactory(graph_).removeAllConnectionsCmd(connectable));
        }
    }

    locked = true;

    if(Meta::doExecute()) {
        saved_state = node_handle->getNodeStateCopy();

        graph_->deleteNode(node_handle->getUUID());
        return true;
    }

    return false;
}

bool DeleteNode::doUndo()
{
    NodeWorker::Ptr node = node_factory_->makeNode(type, uuid);

    node->getNodeHandle()->setNodeState(saved_state);

    graph_->addNode(node);

    return Meta::doUndo();
}

bool DeleteNode::doRedo()
{
    if(Meta::doRedo()) {
        NodeHandle* node_handle = graph_->findNodeHandle(uuid);
        saved_state = node_handle->getNodeStateCopy();

        graph_->deleteNode(node_handle->getUUID());
        return true;
    }

    return false;
}
