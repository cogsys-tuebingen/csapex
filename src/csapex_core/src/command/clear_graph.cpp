/// HEADER
#include <csapex/command/clear_graph.h>

/// COMPONENT
#include <csapex/command/command_factory.h>
#include <csapex/command/delete_node.h>
#include <csapex/model/node_constructor.h>
#include <csapex/model/node.h>
#include <csapex/model/node_facade_impl.h>
#include <csapex/model/graph/graph_impl.h>
#include <csapex/factory/node_factory_impl.h>
#include <csapex/model/subgraph_node.h>
#include <csapex/command/command_serializer.h>
#include <csapex/serialization/io/std_io.h>
#include <csapex/model/graph_facade_impl.h>

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
    GraphFacadeImplementation* graph_facade = graph_uuid.empty() ? getRoot() : getGraphFacade();
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


void ClearGraph::serialize(SerializationBuffer &data, SemanticVersion& version) const
{
    Meta::serialize(data, version);
}

void ClearGraph::deserialize(const SerializationBuffer& data, const SemanticVersion& version)
{
    Meta::deserialize(data, version);
}
