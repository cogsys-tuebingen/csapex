/// HEADER
#include <csapex/command/ungroup_nodes.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/model/graph.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_state.h>
#include <csapex/command/delete_node.h>
#include <csapex/command/add_node.h>
#include <csapex/command/paste_graph.h>
#include <csapex/model/graph_facade.h>
#include <csapex/scheduling/thread_pool.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/model/connection.h>
#include <csapex/core/graphio.h>
#include <csapex/command/pass_out_connector.h>
#include <csapex/command/add_msg_connection.h>
#include <csapex/utility/assert.h>

/// SYSTEM
#include <sstream>
#include <iostream>

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
    subgraph = std::dynamic_pointer_cast<Graph>(nh->getNode().lock());

    apex_assert_hard(subgraph);

    setNodes(subgraph->getAllNodeHandles());

    analyzeConnections(subgraph.get());

    {
        selection_yaml = YAML::Node(YAML::NodeType::Map);

        GraphIO io(subgraph.get(), node_factory_);
        io.setIgnoreForwardingConnections(true);
        io.saveGraph(selection_yaml);
    }

    for(InputPtr in : nh->getAllInputs()) {
        old_connections_in[in->getUUID()] = in->getSource()->getUUID();
    }
    for(OutputPtr out : nh->getAllOutputs()) {
        auto& vec = old_connections_out[out->getUUID()];
        for(ConnectionPtr c : out->getConnections()) {
            Input* in = dynamic_cast<Input*>(c->to());
            if(in) {
                vec.push_back(in->getUUID());
            }
        }
    }


    CommandPtr del = std::make_shared<command::DeleteNode>(getGraphFacade()->getAbsoluteUUID(), uuid);
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
        std::string child = ci.to.id();
        UUID to = UUIDProvider::makeDerivedUUID_forced(nested_node_parent_id, child);

        UUID graph_in = subgraph->getForwardedOutputExternal(ci.from);
        UUID from = old_connections_in[graph_in];

        CommandPtr add_connection = std::make_shared<command::AddMessageConnection>(parent_auuid, from, to);
        executeCommand(add_connection);
        add(add_connection);
    }

    for(const ConnectionInformation& ci : connections_going_out) {
        UUID nested_node_parent_id = old_uuid_to_new.at(ci.from.parentUUID());
        std::string child = ci.from.id();
        UUID from = UUIDProvider::makeDerivedUUID_forced(nested_node_parent_id, child);

        UUID graph_out = subgraph->getForwardedInputExternal(ci.to);
        const std::vector<UUID>& targets = old_connections_out[graph_out];


        for(const UUID& to : targets) {
            CommandPtr add_connection = std::make_shared<command::AddMessageConnection>(parent_auuid, from, to);
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

