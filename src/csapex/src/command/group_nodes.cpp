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
#include <csapex/model/graph_facade.h>
#include <csapex/scheduling/thread_pool.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/model/connection.h>

/// SYSTEM
#include <sstream>

/// COMPONENT
#include <csapex/utility/assert.h>

using namespace csapex::command;

GroupNodes::GroupNodes(const std::vector<UUID> &nodes)
    : Meta("GroupNodes"), uuids(nodes)
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
    if(uuids.empty()) {
        return true;
    }

    std::set<NodeHandle*> node_set;
    std::vector<NodeHandle*> nodes;
    for(const UUID& uuid : uuids) {
        NodeHandle* nh = graph_->findNodeHandle(uuid);
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

    UUID sub_graph_uuid = graph_->generateUUID("subgraph");
    CommandPtr add_graph = std::make_shared<command::AddNode>("csapex::Graph", insert_pos, UUID::NONE, sub_graph_uuid, nullptr);
    add_graph->executeCommand(graph_facade_, graph_, thread_pool_, node_factory_, add_graph);
    add(add_graph);

    NodeHandle* sub_graph_nh = graph_->findNodeHandle(sub_graph_uuid);
    apex_assert_hard(sub_graph_nh);

    NodePtr sub_graph_node = sub_graph_nh->getNode().lock();
    apex_assert_hard(sub_graph_node);

    GraphPtr sub_graph = std::dynamic_pointer_cast<Graph>(sub_graph_node);
    apex_assert_hard(sub_graph);

    GraphFacade* sub_graph_facade = graph_facade_->getSubGraph(sub_graph_uuid);
    apex_assert_hard(sub_graph_facade);

    struct ConnectionInformation {
        UUID from;
        UUID to;
    };

    std::vector<ConnectionInformation> connections_inside;
    std::vector<ConnectionInformation> connections_going_in;
    std::vector<ConnectionInformation> connections_going_out;

    for(NodeHandle* nh : nodes) {
        for(const InputPtr& input : nh->getAllInputs()) {
            for(const ConnectionPtr& connection : input->getConnections()) {
                Output* output = dynamic_cast<Output*>(connection->from());
                apex_assert_hard(output);

                NodeHandle* source = graph_->findNodeHandleForConnector(output->getUUID());
                apex_assert_hard(source);

                ConnectionInformation c;
                c.from = output->getUUID();
                c.to = input->getUUID();

                if(node_set.find(source) != node_set.end()) {
                    // inside
                    connections_inside.push_back(c);
                } else {
                    // coming in
                    connections_going_in.push_back(c);
                }
            }
        }
        for(const OutputPtr& output : nh->getAllOutputs()) {
            for(const ConnectionPtr& connection : output->getConnections()) {
                Input* input = dynamic_cast<Input*>(connection->to());
                apex_assert_hard(input);

                NodeHandle* target = graph_->findNodeHandleForConnector(input->getUUID());
                apex_assert_hard(target);

                if(node_set.find(target) != node_set.end()) {
                    // inside -> already covered
                } else {
                    // going out
                    ConnectionInformation c;
                    c.from = output->getUUID();
                    c.to = input->getUUID();
                    connections_going_out.push_back(c);
                }
            }
        }
    }

    std::unordered_map<UUID, UUID, UUID::Hasher> old_uuid_to_new;

    for(NodeHandle* nh : nodes) {
        std::string type = nh->getType();
        NodeStatePtr state = nh->getNodeStateCopy();
        Point pos = state->getPos();
        UUID new_uuid = sub_graph->generateUUID(type);

        old_uuid_to_new[nh->getUUID()] = new_uuid;

        CommandPtr add_graph = std::make_shared<command::AddNode>(type, pos, UUID::NONE, new_uuid, state);
        add_graph->executeCommand(sub_graph_facade, sub_graph.get(), thread_pool_, node_factory_, add_graph);
    }

    for(NodeHandle* nh : nodes) {
        UUID old_uuid = nh->getUUID();
        CommandPtr del = std::make_shared<command::DeleteNode>(old_uuid);
        del->executeCommand(graph_facade_, graph_, thread_pool_, node_factory_, del);
        add(del);
    }

    for(const ConnectionInformation& ci : connections_going_in) {
        UUID parent_mapped = old_uuid_to_new[ci.to.parentUUID()];
        std::string child = ci.to.id();

        UUID new_uuid = UUIDProvider::makeDerivedUUID_forced(parent_mapped, child);

        UUID relay_input = sub_graph->passOutInput(new_uuid);

        CommandPtr add_connection = std::make_shared<command::AddMessageConnection>(ci.from, relay_input);
        add_connection->executeCommand(graph_facade_, graph_, thread_pool_, node_factory_, add_connection);

        add(add_connection);
    }

    for(const ConnectionInformation& ci : connections_going_out) {
        UUID parent_mapped = old_uuid_to_new[ci.from.parentUUID()];
        std::string child = ci.from.id();

        UUID new_uuid = UUIDProvider::makeDerivedUUID_forced(parent_mapped, child);

        UUID relay_input = sub_graph->passOutOutput(new_uuid);

        CommandPtr add_connection = std::make_shared<command::AddMessageConnection>(relay_input, ci.to);
        add_connection->executeCommand(graph_facade_, graph_, thread_pool_, node_factory_, add_connection);

        add(add_connection);
    }


    for(const ConnectionInformation& ci : connections_inside) {
        UUID from_parent_mapped = old_uuid_to_new[ci.from.parentUUID()];
        std::string from_child = ci.from.id();

        UUID from_new_uuid = UUIDProvider::makeDerivedUUID_forced(from_parent_mapped, from_child);

        UUID to_parent_mapped = old_uuid_to_new[ci.to.parentUUID()];
        std::string to_child = ci.to.id();

        UUID to_new_uuid = UUIDProvider::makeDerivedUUID_forced(to_parent_mapped, to_child);

        CommandPtr add_connection = std::make_shared<command::AddMessageConnection>(from_new_uuid, to_new_uuid);
        add_connection->executeCommand(sub_graph_facade, sub_graph.get(), thread_pool_, node_factory_, add_connection);

        add(add_connection);
    }

    return true;
}

bool GroupNodes::doUndo()
{
    return Meta::doUndo();
}

bool GroupNodes::doRedo()
{
    return doExecute();
}

