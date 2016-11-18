/// HEADER
#include <csapex/command/ungroup_nodes.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/model/subgraph_node.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_state.h>
#include <csapex/command/delete_node.h>
#include <csapex/command/add_node.h>
#include <csapex/command/paste_graph.h>
#include <csapex/command/command_factory.h>
#include <csapex/model/graph_facade.h>
#include <csapex/scheduling/thread_pool.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/signal/event.h>
#include <csapex/signal/slot.h>
#include <csapex/model/connection.h>
#include <csapex/core/graphio.h>
#include <csapex/command/add_variadic_connector.h>
#include <csapex/command/add_connection.h>
#include <csapex/utility/assert.h>

/// SYSTEM
#include <sstream>
#include <iostream>

using namespace csapex;
using namespace csapex::command;

UngroupNodes::UngroupNodes(const AUUID& parent_uuid, const UUID &uuid)
    : GroupBase(parent_uuid, "UngroupNodes"), uuid(uuid)
{
}

std::string UngroupNodes::getType() const
{
    return "UngroupNodes";
}

std::string UngroupNodes::getDescription() const
{
    return "inline a sub graph";
}

bool UngroupNodes::doExecute()
{
    Graph* graph = getGraph();

    NodeHandle* nh = graph->findNodeHandle(uuid);
    subgraph = std::dynamic_pointer_cast<SubgraphNode>(nh->getNode().lock());

    apex_assert_hard(subgraph);

    setNodes(subgraph->getAllNodeHandles());

    analyzeConnections(subgraph.get());

    {
        GraphIO io(subgraph.get(), getNodeFactory());
        io.setIgnoreForwardingConnections(true);
        serialized_snippet_ = io.saveGraph();
    }

    for(InputPtr in : nh->getExternalInputs()) {
        auto source = in->getSource();
        if(source) {
            old_connections_in[in->getUUID()] = source->getUUID();
        }
    }
    for(OutputPtr out : nh->getExternalOutputs()) {
        auto& vec = old_connections_out[out->getUUID()];
        for(ConnectionPtr c : out->getConnections()) {
            vec.push_back(c->to()->getUUID());
        }
    }


    for(SlotPtr slot : nh->getExternalSlots()) {
        auto& vec = old_signals_in[slot->getUUID()];
        for(ConnectionPtr c : slot->getConnections()) {
            vec.push_back(c->from()->getUUID());
        }
    }

    for(EventPtr trigger : nh->getExternalEvents()) {
        auto& vec = old_signals_out[trigger->getUUID()];
        for(ConnectionPtr c : trigger->getConnections()) {
            vec.push_back(c->to()->getUUID());
        }
    }

    CommandFactory cf(graph_facade_);

    CommandPtr del = cf.deleteAllNodes({uuid});
    executeCommand(del);
    add(del);

    insert_pos = nh->getNodeState()->getPos();

    pasteSelection(graph_uuid);

    unmapConnections(graph_uuid, uuid.getAbsoluteUUID());

    subgraph.reset();

    return true;
}


void UngroupNodes::unmapConnections(AUUID parent_auuid, AUUID sub_graph_auuid)
{
    for(const ConnectionInformation& ci : connections_going_in) {
        UUID nested_node_parent_id = old_uuid_to_new.at(ci.to.parentUUID());
        std::string child = ci.to.id().getFullName();
        UUID to = UUIDProvider::makeDerivedUUID_forced(nested_node_parent_id, child);

        UUID graph_in = subgraph->getForwardedOutputExternal(ci.from);
        UUID from = old_connections_in[graph_in];

        CommandPtr add_connection = std::make_shared<command::AddConnection>(parent_auuid, from, to, ci.active);
        executeCommand(add_connection);
        add(add_connection);
    }

    for(const ConnectionInformation& ci : connections_going_out) {
        UUID nested_node_parent_id = old_uuid_to_new.at(ci.from.parentUUID());
        std::string child = ci.from.id().getFullName();
        UUID from = UUIDProvider::makeDerivedUUID_forced(nested_node_parent_id, child);

        UUID graph_out = subgraph->getForwardedInputExternal(ci.to);
        const std::vector<UUID>& targets = old_connections_out[graph_out];


        for(const UUID& to : targets) {
            CommandPtr add_connection = std::make_shared<command::AddConnection>(parent_auuid, from, to, ci.active);
            executeCommand(add_connection);
            add(add_connection);
        }
    }

    for(const ConnectionInformation& ci : signals_going_in) {
        UUID nested_node_parent_id = old_uuid_to_new.at(ci.to.parentUUID());
        std::string child = ci.to.id().getFullName();
        UUID to = UUIDProvider::makeDerivedUUID_forced(nested_node_parent_id, child);

        UUID graph_out = subgraph->getForwardedSlotExternal(ci.from);
        const std::vector<UUID>& targets = old_signals_in[graph_out];


        for(const UUID& from : targets) {
            CommandPtr add_connection = std::make_shared<command::AddConnection>(parent_auuid, from, to, ci.active);
            executeCommand(add_connection);
            add(add_connection);
        }
    }

    for(const ConnectionInformation& ci : signals_going_out) {
        UUID nested_node_parent_id = old_uuid_to_new.at(ci.from.parentUUID());
        std::string child = ci.from.id().getFullName();
        UUID from = UUIDProvider::makeDerivedUUID_forced(nested_node_parent_id, child);

        UUID graph_out = subgraph->getForwardedEventExternal(ci.to);
        const std::vector<UUID>& targets = old_signals_out[graph_out];


        for(const UUID& to : targets) {
            CommandPtr add_connection = std::make_shared<command::AddConnection>(parent_auuid, from, to, ci.active);
            executeCommand(add_connection);
            add(add_connection);
        }
    }
}
bool UngroupNodes::doUndo()
{
    return Meta::doUndo();
}

bool UngroupNodes::doRedo()
{
    locked = false;
    clear();
    return doExecute();
}

