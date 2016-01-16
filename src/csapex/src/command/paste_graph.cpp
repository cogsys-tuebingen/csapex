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

using namespace csapex;
using namespace csapex::command;

PasteGraph::PasteGraph(const YAML::Node& blueprint, const Point& pos)
    : blueprint_(blueprint), pos_(pos)
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
    bool paused = graph_facade_->isPaused();
    graph_facade_->pauseRequest(true);

    GraphIO io(graph_, node_factory_);

    inserted_ = io.loadIntoGraph(blueprint_, pos_);

    graph_facade_->pauseRequest(paused);

    return true;
}

bool PasteGraph::doUndo()
{
    for(const UUID& id : inserted_) {
        CommandPtr del(new command::DeleteNode(id));
        del->executeCommand(graph_facade_, graph_, thread_pool_, node_factory_, del);
    }

    inserted_.clear();

    return true;
}

bool PasteGraph::doRedo()
{
    return doExecute();
}
