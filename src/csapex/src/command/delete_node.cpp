/// HEADER
#include <csapex/command/delete_node.h>

/// COMPONENT
#include <csapex/command/delete_connection.h>
#include <csapex/model/node_constructor.h>
#include <csapex/model/node.h>
#include <csapex/model/node_worker.h>
#include <csapex/model/node_state.h>
#include <csapex/model/node_factory.h>
#include <csapex/model/graph.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/view/widget_controller.h>

/// SYSTEM
#include <boost/foreach.hpp>

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
    NodeWorker* node_worker = graph_->findNodeWorkerForConnector(uuid);

    type = node_worker->getType();

    locked = false;
    clear();

    foreach(Connectable* i, node_worker->getAllConnectors()) {
        if(i->isConnected()) {
            add(i->removeAllConnectionsCmd());
        }
    }

    locked = true;

    if(Meta::doExecute()) {
        saved_state = node_worker->getNodeStateCopy();

        graph_->deleteNode(node_worker->getUUID());
        return true;
    }

    return false;
}

bool DeleteNode::doUndo()
{
    NodeWorker::Ptr node = widget_ctrl_->getNodeFactory()->makeNode(type, uuid);

    node->setNodeState(saved_state);

    graph_->addNode(node);

    return Meta::doUndo();
}

bool DeleteNode::doRedo()
{
    if(Meta::doRedo()) {
        NodeWorker* node_worker = graph_->findNodeWorker(uuid);
        saved_state = node_worker->getNodeStateCopy();

        graph_->deleteNode(node_worker->getUUID());
        return true;
    }

    return false;
}
