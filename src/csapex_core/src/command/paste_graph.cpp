/// HEADER
#include <csapex/command/paste_graph.h>

/// COMPONENT
#include <csapex/command/delete_node.h>
#include <csapex/utility/assert.h>
#include <csapex/core/graphio.h>
#include <csapex/model/graph_facade_impl.h>
#include <csapex/command/command_factory.h>
#include <csapex/command/command_serializer.h>
#include <csapex/serialization/io/std_io.h>
#include <csapex/serialization/snippet.h>

using namespace csapex;
using namespace csapex::command;

CSAPEX_REGISTER_COMMAND_SERIALIZER(PasteGraph)

PasteGraph::PasteGraph(const AUUID& graph_id, const Snippet& blueprint, const Point& pos) : CommandImplementation(graph_id), blueprint_(std::make_shared<Snippet>(blueprint)), pos_(pos)
{
}
std::string PasteGraph::getDescription() const
{
    return std::string("paste into a graph");
}

bool PasteGraph::doExecute()
{
    GraphFacadeImplementation* graph_facade = graph_uuid.empty() ? getRoot() : getGraphFacade();
    bool paused = graph_facade->isPaused();
    graph_facade->pauseRequest(true);

    GraphIO io(*graph_facade, getNodeFactory());

    id_mapping_ = io.loadIntoGraph(*blueprint_, pos_);

    graph_facade->pauseRequest(paused);

    return true;
}

bool PasteGraph::doUndo()
{
    if (!delete_command_) {
        std::vector<UUID> uuids;

        GraphFacade* graph_facade = graph_uuid.empty() ? getRoot() : getGraphFacade();
        for (const auto& pair : id_mapping_) {
            uuids.push_back(pair.second);
        }

        delete_command_ = CommandFactory(graph_facade).deleteAllNodes(uuids);
    }

    executeCommand(delete_command_);

    return true;
}

bool PasteGraph::doRedo()
{
    return undoCommand(delete_command_);
}

std::unordered_map<UUID, UUID, UUID::Hasher> PasteGraph::getMapping() const
{
    return id_mapping_;
}

void PasteGraph::serialize(SerializationBuffer& data, SemanticVersion& version) const
{
    data << blueprint_;
    data << pos_.x << pos_.y;
}

void PasteGraph::deserialize(const SerializationBuffer& data, const SemanticVersion& version)
{
    data >> blueprint_;
    data >> pos_.x >> pos_.y;
}
