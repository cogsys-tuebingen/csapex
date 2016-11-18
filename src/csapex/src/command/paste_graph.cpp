/// HEADER
#include <csapex/command/paste_graph.h>

/// COMPONENT
#include <csapex/command/delete_node.h>
#include <csapex/model/node_constructor.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/model/graph.h>
#include <csapex/utility/assert.h>
#include <csapex/core/graphio.h>
#include <csapex/model/graph_facade.h>
#include <csapex/command/add_variadic_connector.h>
#include <csapex/command/add_connection.h>
#include <csapex/command/command_factory.h>

using namespace csapex;
using namespace csapex::command;

PasteGraph::PasteGraph(const AUUID &graph_id, const Snippet &blueprint, const Point& pos)
    : Meta(graph_id, "PasteGraph"), blueprint_(blueprint), pos_(pos)
{
}

std::string PasteGraph::getType() const
{
    return "PasteGraph";
}

std::string PasteGraph::getDescription() const
{
    return std::string("paste into a graph");
}

bool PasteGraph::doExecute()
{
    GraphFacade* graph_facade = graph_uuid.empty() ? getRoot() : getGraphFacade();
    bool paused = graph_facade->isPaused();
    graph_facade->pauseRequest(true);

    SubgraphNode* graph = graph_facade->getSubgraphNode();

    GraphIO io(graph, getNodeFactory());

    id_mapping_ = io.loadIntoGraph(blueprint_, pos_);

    graph_facade->pauseRequest(paused);

    return true;
}

bool PasteGraph::doUndo()
{
    std::vector<UUID> uuids;

    GraphFacade* graph_facade = graph_uuid.empty() ? getRoot() : getGraphFacade();
    for(const auto& pair : id_mapping_) {
        uuids.push_back(pair.second);
//        CommandPtr del(new command::DeleteNode(graph_uuid, pair.second));
//        del->init(graph_facade, *core_, getDesigner());
//        executeCommand(del);
    }

    CommandFactory cf(graph_facade);

    CommandPtr del = cf.deleteAllNodes(uuids);
    //        del->init(graph_facade, *core_, getDesigner());
    executeCommand(del);


    id_mapping_.clear();

    Meta::doUndo();

    return true;
}

bool PasteGraph::doRedo()
{
    clear();
    return doExecute();
}

std::unordered_map<UUID, UUID, UUID::Hasher> PasteGraph::getMapping() const
{
    return id_mapping_;
}
