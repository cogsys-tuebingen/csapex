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
#include <csapex/model/graph_worker.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/view/widget_controller.h>

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
    NodeWorker* node_worker = graph_worker_->getGraph()->findNodeWorkerForConnector(uuid);

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

        graph_worker_->getGraph()->deleteNode(node_worker->getUUID());
        return true;
    }

    return false;
}

bool DeleteNode::doUndo()
{
    NodeWorker::Ptr node = node_factory_->makeNode(type, uuid);

    node->setNodeState(saved_state);
    node->pause(graph_worker_->isPaused());

    graph_worker_->getGraph()->addNode(node);

    return Meta::doUndo();
}

bool DeleteNode::doRedo()
{
    if(Meta::doRedo()) {
        NodeWorker* node_worker = graph_worker_->getGraph()->findNodeWorker(uuid);
        saved_state = node_worker->getNodeStateCopy();

        graph_worker_->getGraph()->deleteNode(node_worker->getUUID());
        return true;
    }

    return false;
}
