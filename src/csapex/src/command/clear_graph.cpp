/// HEADER
#include <csapex/command/clear_graph.h>

/// COMPONENT
#include <csapex/command/command_factory.h>
#include <csapex/command/delete_node.h>
#include <csapex/model/node_constructor.h>
#include <csapex/model/node.h>
#include <csapex/model/node_facade_local.h>
#include <csapex/model/graph/graph_local.h>
#include <csapex/factory/node_factory.h>
#include <csapex/model/subgraph_node.h>
#include <csapex/command/command_serializer.h>
#include <csapex/serialization/serialization_buffer.h>
#include <csapex/model/graph_facade_local.h>

/// SYSTEM

using namespace csapex;
using namespace csapex::command;

CSAPEX_REGISTER_COMMAND_SERIALIZER(ClearGraph)

ClearGraph::ClearGraph(const AUUID& parent_uuid)
    : Meta(parent_uuid, "clear graph")
{
}

std::string ClearGraph::getDescription() const
{
    return std::string("clear graph ") + graph_uuid.getFullName();
}


bool ClearGraph::doExecute()
{
    GraphFacadeLocal* graph_facade = graph_uuid.empty() ? getRoot() : getGraphFacade();
    bool paused = graph_facade->isPaused();
    graph_facade->pauseRequest(true);

    add(CommandFactory(graph_facade).deleteAllNodes(graph_facade->enumerateAllNodes()));

    if(Meta::doExecute()) {
        graph_facade->pauseRequest(paused);
        return true;
    }


    return false;
}

bool ClearGraph::doUndo()
{
    return Meta::doUndo();
}

bool ClearGraph::doRedo()
{
    return Meta::doRedo();
}


void ClearGraph::serialize(SerializationBuffer &data) const
{
    Meta::serialize(data);
}

void ClearGraph::deserialize(const SerializationBuffer& data)
{
    Meta::deserialize(data);
}
