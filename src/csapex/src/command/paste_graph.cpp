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
#include <csapex/command/pass_out_connector.h>
#include <csapex/command/add_msg_connection.h>

using namespace csapex;
using namespace csapex::command;

PasteGraph::PasteGraph(const UUID &graph_id, const YAML::Node& blueprint, const Point& pos)
    : Meta("PasteGraph"), graph_id_(graph_id), blueprint_(blueprint), pos_(pos)
{
}

PasteGraph::PasteGraph(const UUID& graph_id, const YAML::Node& blueprint, const Point &pos,
                       const std::vector<std::pair<UUID, UUID> > &crossing_inputs, const std::vector<std::pair<UUID, UUID> > &crossing_outputs)
    : PasteGraph(graph_id, blueprint, pos)
{
    crossing_inputs_ = crossing_inputs;
    crossing_outputs_ = crossing_outputs;
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
    GraphFacade* graph_facade = graph_id_.empty() ? getRoot() : getSubGraph(graph_id_);
    bool paused = graph_facade->isPaused();
    graph_facade->pauseRequest(true);

    Graph* graph = graph_facade->getGraph();

    GraphIO io(graph, node_factory_);

    id_mapping_ = io.loadIntoGraph(blueprint_, pos_);

    graph_facade->pauseRequest(paused);

    for(const std::pair<UUID,UUID>& in : crossing_inputs_) {
        UUID parent_mapped = id_mapping_[in.second.parentUUID()];
        std::string child = in.second.id();

        UUID new_uuid = graph->makeDerivedUUID(parent_mapped, child);
        CommandPtr pass_out = std::make_shared<command::PassOutConnector>(graph_id_, new_uuid);
        pass_out->init(settings_, getRoot(), getRootThreadPool(), node_factory_);
        executeCommand(pass_out);
        add(pass_out);
    }

    for(const std::pair<UUID,UUID>& out : crossing_outputs_) {
        UUID parent_mapped = id_mapping_[out.first.parentUUID()];
        std::string child = out.first.id();

        UUID new_uuid = graph->makeDerivedUUID(parent_mapped, child);
        CommandPtr pass_out = std::make_shared<command::PassOutConnector>(graph_id_, new_uuid);
        pass_out->init(settings_, getRoot(), getRootThreadPool(), node_factory_);
        executeCommand(pass_out);
        add(pass_out);
    }


    for(const std::pair<UUID,UUID>& in : crossing_inputs_) {
        UUID parent_mapped = id_mapping_[in.second.parentUUID()];
        std::string child = in.second.id();

        UUID new_uuid = graph->makeDerivedUUID(parent_mapped, child);
        UUID forwarding_uuid = graph_facade->getGraph()->getForwardingInput(new_uuid);

        CommandPtr add_connection = std::make_shared<command::AddMessageConnection>(in.first, forwarding_uuid);
        add_connection->init(settings_, getRoot(), getRootThreadPool(), node_factory_);
        executeCommand(add_connection);
        add(add_connection);
    }

    for(const std::pair<UUID,UUID>& out : crossing_outputs_) {
        UUID parent_mapped = id_mapping_[out.first.parentUUID()];
        std::string child = out.first.id();

        UUID new_uuid = graph->makeDerivedUUID(parent_mapped, child);
        UUID forwarding_uuid = graph_facade->getGraph()->getForwardingOutput(new_uuid);

        CommandPtr add_connection = std::make_shared<command::AddMessageConnection>(forwarding_uuid, out.second);
        add_connection->init(settings_, getRoot(), getRootThreadPool(), node_factory_);
        executeCommand(add_connection);
        add(add_connection);
    }


    return true;
}

bool PasteGraph::doUndo()
{
    GraphFacade* graph_facade = graph_id_.empty() ? getRoot() : getSubGraph(graph_id_);
    for(const auto& pair : id_mapping_) {
        CommandPtr del(new command::DeleteNode(pair.second));
        del->init(settings_, graph_facade, getRootThreadPool(), node_factory_);
        executeCommand(del);
    }

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
