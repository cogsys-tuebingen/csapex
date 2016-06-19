/// HEADER
#include <csapex/command/delete_node.h>

/// COMPONENT
#include <csapex/command/delete_msg_connection.h>
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
#include <csapex/model/connection.h>
#include <csapex/core/graphio.h>

/// SYSTEM

using namespace csapex::command;

DeleteNode::DeleteNode(const AUUID& parent_uuid, const UUID& uuid)
    : Meta(parent_uuid, "delete node and connections"), uuid(uuid)
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
    Graph* graph = getGraph();
    NodeHandle* node_handle = graph->findNodeHandle(uuid);

    type = node_handle->getType();

    locked = false;
    clear();

    std::set<ConnectionPtr> connections;
    for(ConnectablePtr connectable : node_handle->getExternalConnectors()) {
        if(connectable->isConnected()) {
            for(ConnectionPtr c : connectable->getConnections()) {
                connections.insert(c);
            }
        }
    }

    for(ConnectionPtr c : connections) {
        add(std::make_shared<DeleteMessageConnection>(graph_uuid, c->from(), c->to()));
    }

    // serialize sub graph
    if(node_handle->getType() == "csapex::Graph") {
        GraphPtr g = std::dynamic_pointer_cast<Graph>(node_handle->getNode().lock());
        apex_assert_hard(g);

        GraphIO io(g.get(), getNodeFactory());
        io.saveGraph(saved_graph);
    }

    locked = true;

    if(Meta::doExecute()) {
        saved_state = node_handle->getNodeStateCopy();

        graph->deleteNode(node_handle->getUUID());
        return true;
    }

    return false;
}

bool DeleteNode::doUndo()
{
    Graph* graph = getGraph();
    NodeHandlePtr node_handle = getNodeFactory()->makeNode(type, uuid, graph);
    node_handle->setNodeState(saved_state);

    graph->addNode(node_handle);

    //deserialize subgraph
    if(node_handle->getType() == "csapex::Graph") {
        GraphPtr g = std::dynamic_pointer_cast<Graph>(node_handle->getNode().lock());
        apex_assert_hard(g);

        GraphIO io(g.get(), getNodeFactory());
        io.loadGraph(saved_graph);
    }

    return Meta::doUndo();
}

bool DeleteNode::doRedo()
{
    if(Meta::doRedo()) {
        Graph* graph = getGraph();
        NodeHandle* node_handle = graph->findNodeHandle(uuid);
        saved_state = node_handle->getNodeStateCopy();

        graph->deleteNode(node_handle->getUUID());
        return true;
    }

    return false;
}
