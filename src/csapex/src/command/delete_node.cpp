/// HEADER
#include <csapex/command/delete_node.h>

/// COMPONENT
#include <csapex/command/command_factory.h>
#include <csapex/model/node_constructor.h>
#include <csapex/model/node.h>
#include <csapex/model/node_facade_local.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_worker.h>
#include <csapex/model/node_state.h>
#include <csapex/model/graph/graph_local.h>
#include <csapex/factory/node_factory_local.h>
#include <csapex/model/subgraph_node.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/model/connection.h>
#include <csapex/core/graphio.h>
#include <csapex/command/command_serializer.h>
#include <csapex/serialization/serialization_buffer.h>
#include <csapex/model/graph_facade_local.h>

/// SYSTEM

using namespace csapex;
using namespace csapex::command;

CSAPEX_REGISTER_COMMAND_SERIALIZER(DeleteNode)

DeleteNode::DeleteNode(const AUUID& parent_uuid, const UUID& uuid)
    : Meta(parent_uuid, "delete node and connections"), uuid(uuid)
{
}

std::string DeleteNode::getDescription() const
{
    return std::string("deleted node ") + uuid.getFullName();
}


bool DeleteNode::doExecute()
{
    GraphLocalPtr graph = getGraph();
    NodeHandle* node_handle = graph->findNodeHandle(uuid);

    type = node_handle->getType();

    locked = false;
    clear();

    for(ConnectablePtr connectable : node_handle->getExternalConnectors()) {
        apex_assert(!connectable->isConnected());
    }

    // serialize sub graph
    if(node_handle->isGraph()) {
        GraphFacadeLocalPtr g = root_graph_facade_->getLocalSubGraph(node_handle->getUUID());
        apex_assert_hard(g);

        GraphIO io(*g, getNodeFactory());
        saved_graph = io.saveGraph();
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
    GraphLocalPtr graph = getGraph();
    NodeFacadeLocalPtr node_facade = getNodeFactory()->makeNode(type, uuid, graph);
    node_facade->setNodeState(saved_state);

    graph->addNode(node_facade);

    //deserialize subgraph
    if(node_facade->isGraph()) {
        GraphFacadeLocalPtr g = root_graph_facade_->getLocalSubGraph(node_facade->getUUID());
        apex_assert_hard(g);

        GraphIO io(*g, getNodeFactory());
        io.loadGraph(saved_graph);
    }

    return Meta::doUndo();
}

bool DeleteNode::doRedo()
{
    if(Meta::doRedo()) {
        GraphLocalPtr graph = getGraph();
        NodeHandle* node_handle = graph->findNodeHandle(uuid);
        saved_state = node_handle->getNodeStateCopy();

        graph->deleteNode(node_handle->getUUID());
        return true;
    }

    return false;
}


void DeleteNode::serialize(SerializationBuffer &data) const
{
    Meta::serialize(data);

    data << uuid;
    data << type;
}

void DeleteNode::deserialize(const SerializationBuffer& data)
{
    Meta::deserialize(data);

    data >> uuid;
    data >> type;
}
