/// HEADER
#include <csapex/command/group_nodes.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/model/graph.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_state.h>
#include <csapex/command/delete_node.h>
#include <csapex/command/add_node.h>
#include <csapex/command/add_msg_connection.h>
#include <csapex/command/paste_graph.h>
#include <csapex/command/pass_out_connector.h>
#include <csapex/model/graph_facade.h>
#include <csapex/scheduling/thread_pool.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/model/connection.h>
#include <csapex/core/graphio.h>

/// SYSTEM
#include <sstream>

/// COMPONENT
#include <csapex/utility/assert.h>

using namespace csapex::command;

GroupNodes::GroupNodes(const AUUID& parent_uuid, const std::vector<UUID> &uuids)
    : Meta(parent_uuid, "GroupNodes"), uuids(uuids)
{
}

std::string GroupNodes::getType() const
{
    return "GroupNodes";
}

std::string GroupNodes::getDescription() const
{
    return "create a sub graph";
}

bool GroupNodes::doExecute()
{
    Graph* graph = getGraph();
    {
        GraphIO io(graph, node_factory_);
        selection_yaml = YAML::Node(YAML::NodeType::Map);
        io.saveSelectedGraph(selection_yaml, uuids);
    }

    std::set<NodeHandle*> node_set;
    std::vector<NodeHandle*> nodes;
    for(const UUID& uuid : uuids) {
        NodeHandle* nh = graph->findNodeHandle(uuid);
        nodes.push_back(nh);
        node_set.insert(nh);
    }

    apex_assert_hard(!nodes.empty());

    Point insert_pos = nodes[0]->getNodeState()->getPos();
    for(NodeHandle* nh : nodes) {
        Point pos = nh->getNodeState()->getPos();
        if(pos.x < insert_pos.x) {
            insert_pos.x = pos.x;
        }
        if(pos.y < insert_pos.y) {
            insert_pos.y = pos.y;
        }
    }

    if(sub_graph_uuid_.empty()) {
        sub_graph_uuid_ = graph->generateUUID("csapex::Graph");
    }

    AUUID parent_auuid = getGraphFacade()->getAbsoluteUUID();
    AUUID sub_graph_auuid(UUIDProvider::makeDerivedUUID_forced(
                              parent_auuid,
                              sub_graph_uuid_.getFullName()));

    CommandPtr add_graph = std::make_shared<command::AddNode>(parent_auuid, "csapex::Graph", insert_pos, sub_graph_uuid_, nullptr);
    executeCommand(add_graph);
    add(add_graph);

//    NodeHandle* sub_graph_nh = graph_->findNodeHandle(sub_graph_uuid);
//    apex_assert_hard(sub_graph_nh);

//    NodePtr sub_graph_node = sub_graph_nh->getNode().lock();
//    apex_assert_hard(sub_graph_node);

//    GraphPtr sub_graph = std::dynamic_pointer_cast<Graph>(sub_graph_node);
//    apex_assert_hard(sub_graph);

//    GraphFacade* sub_graph_facade = graph_facade_->getSubGraph(sub_graph_uuid);
//    apex_assert_hard(sub_graph_facade);

    struct ConnectionInformation {
        UUID from;
        UUID to;
        ConnectionTypeConstPtr type;
    };

    std::vector<ConnectionInformation> connections_going_in;
    std::vector<ConnectionInformation> connections_going_out;

    std::vector<std::pair<UUID, UUID>> crossing_inputs;
    std::vector<std::pair<UUID, UUID>> crossing_outputs;

    for(NodeHandle* nh : nodes) {
        for(const InputPtr& input : nh->getAllInputs()) {
            for(const ConnectionPtr& connection : input->getConnections()) {
                Output* output = dynamic_cast<Output*>(connection->from());
                apex_assert_hard(output);

                if(input->isVirtual() || output->isVirtual()) {
                    continue;
                }

                NodeHandle* source = graph->findNodeHandleForConnector(output->getUUID());
                apex_assert_hard(source);

                ConnectionInformation c;
                c.from = output->getUUID();
                c.to = input->getUUID();
                c.type = output->getType();

                if(node_set.find(source) == node_set.end()) {
                    // coming in
                    connections_going_in.push_back(c);
                    crossing_inputs.push_back({ output->getUUID(), input->getUUID() });
                }
            }
        }
        for(const OutputPtr& output : nh->getAllOutputs()) {
            for(const ConnectionPtr& connection : output->getConnections()) {
                Input* input = dynamic_cast<Input*>(connection->to());
                apex_assert_hard(input);

                if(input->isVirtual() || output->isVirtual()) {
                    continue;
                }

                NodeHandle* target = graph->findNodeHandleForConnector(input->getUUID());
                apex_assert_hard(target);

                if(node_set.find(target) == node_set.end()) {
                    // going out
                    ConnectionInformation c;
                    c.from = output->getUUID();
                    c.to = input->getUUID();
                    c.type = input->getType();
                    connections_going_out.push_back(c);
                    crossing_outputs.push_back({ output->getUUID(), input->getUUID() });
                }
            }
        }
    }

    for(NodeHandle* nh : nodes) {
        UUID old_uuid = nh->getUUID();
        CommandPtr del = std::make_shared<command::DeleteNode>(getGraphFacade()->getAbsoluteUUID(), old_uuid);
        executeCommand(del);
        add(del);
    }



    std::shared_ptr<PasteGraph> paste(new command::PasteGraph(sub_graph_auuid, selection_yaml, insert_pos));
    executeCommand(paste);
    add(paste);

    auto old_uuid_to_new = paste->getMapping();

    for(const ConnectionInformation& ci : connections_going_in) {
        UUID nested_node_parent_id = old_uuid_to_new[ci.to.parentUUID()];
        std::string child = ci.to.id();

        UUID nested_connector_uuid = UUIDProvider::makeDerivedUUID_forced(nested_node_parent_id, child);

        std::shared_ptr<command::PassOutConnector> pass_out =
                std::make_shared<command::PassOutConnector>(sub_graph_auuid, "in", ci.type);
        executeCommand(pass_out);
        add(pass_out);

        std::pair<UUID, UUID> in_map = pass_out->getMap();

        // forwarding connection
        CommandPtr add_internal_connection =
                std::make_shared<command::AddMessageConnection>(sub_graph_auuid, in_map.second, nested_connector_uuid);
        executeCommand(add_internal_connection);
        add(add_internal_connection);

        // crossing connection
        CommandPtr add_external_connection =
                std::make_shared<command::AddMessageConnection>(parent_auuid, ci.from, in_map.first);
        executeCommand(add_external_connection);
        add(add_external_connection);
    }

    for(const ConnectionInformation& ci : connections_going_out) {
        UUID nested_node_parent_id = old_uuid_to_new[ci.from.parentUUID()];
        std::string child = ci.from.id();

        UUID nested_connector_uuid = UUIDProvider::makeDerivedUUID_forced(nested_node_parent_id, child);

        std::shared_ptr<command::PassOutConnector> pass_out =
                std::make_shared<command::PassOutConnector>(sub_graph_auuid, "out", ci.type);
        executeCommand(pass_out);
        add(pass_out);

        std::pair<UUID, UUID> out_map = pass_out->getMap();

        // forwarding connection
        CommandPtr add_internal_connection =
                std::make_shared<command::AddMessageConnection>(sub_graph_auuid, nested_connector_uuid, out_map.second);
        executeCommand(add_internal_connection);
        add(add_internal_connection);

        // crossing connection
        CommandPtr add_external_connection =
                std::make_shared<command::AddMessageConnection>(parent_auuid, out_map.first, ci.to);
        executeCommand(add_external_connection);
        add(add_external_connection);
    }

    //Meta::doExecute();
    return true;
}

bool GroupNodes::doUndo()
{
    return Meta::doUndo();
}

bool GroupNodes::doRedo()
{
    // problem: uses the graph, not the subgraph!
    locked = false;
    clear();
    return doExecute();
}

