/// HEADER
#include <csapex/command/group_nodes.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/model/graph.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_state.h>
#include <csapex/command/delete_node.h>
#include <csapex/command/add_node.h>
#include <csapex/model/graph_facade.h>
#include <csapex/scheduling/thread_pool.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/model/connection.h>
#include <csapex/core/graphio.h>
#include <csapex/command/add_variadic_connector.h>
#include <csapex/command/add_msg_connection.h>
#include <csapex/utility/assert.h>

/// SYSTEM
#include <sstream>

using namespace csapex::command;

GroupNodes::GroupNodes(const AUUID& parent_uuid, const std::vector<UUID> &uuids)
    : GroupBase(parent_uuid, "GroupNodes"), uuids(uuids)
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
        selection_yaml = YAML::Node(YAML::NodeType::Map);

        GraphIO io(graph, node_factory_);
        io.setIgnoreForwardingConnections(true);
        io.saveSelectedGraph(selection_yaml, uuids);
    }


    findNodes(graph);

    apex_assert_hard(!nodes.empty());

    insert_pos = findTopLeftPoint();

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


    analyzeConnections(graph);

    for(NodeHandle* nh : nodes) {
        UUID old_uuid = nh->getUUID();
        CommandPtr del = std::make_shared<command::DeleteNode>(getGraphFacade()->getAbsoluteUUID(), old_uuid);
        executeCommand(del);
        add(del);
    }

    pasteSelection(sub_graph_auuid);

    mapConnections(parent_auuid, sub_graph_auuid);

    return true;
}

void GroupNodes::findNodes(Graph* graph)
{
    std::vector<NodeHandle*> n;
    for(const UUID& uuid : uuids) {
        NodeHandle* nh = graph->findNodeHandle(uuid);
        n.push_back(nh);
    }

    setNodes(n);
}

void GroupNodes::mapConnections(AUUID parent_auuid, AUUID sub_graph_auuid)
{
    for(const ConnectionInformation& ci : connections_going_in) {
        UUID nested_node_parent_id = old_uuid_to_new.at(ci.to.parentUUID());
        std::string child = ci.to.id();
        UUID nested_connector_uuid = UUIDProvider::makeDerivedUUID_forced(nested_node_parent_id, child);

        std::shared_ptr<command::AddVariadicConnector> pass_out =
                std::make_shared<command::AddVariadicConnector>(sub_graph_auuid, ConnectorType::INPUT, ci.type);
        executeCommand(pass_out);
        add(pass_out);

        RelayMapping in_map = pass_out->getMap();

        // forwarding connection
        CommandPtr add_internal_connection =
                std::make_shared<command::AddMessageConnection>(sub_graph_auuid, in_map.internal, nested_connector_uuid);
        executeCommand(add_internal_connection);
        add(add_internal_connection);

        // crossing connection
        CommandPtr add_external_connection =
                std::make_shared<command::AddMessageConnection>(parent_auuid, ci.from, in_map.external);
        executeCommand(add_external_connection);
        add(add_external_connection);
    }

    for(const ConnectionInformation& ci : connections_going_out) {
        UUID nested_node_parent_id = old_uuid_to_new.at(ci.from.parentUUID());
        std::string child = ci.from.id();
        UUID nested_connector_uuid = UUIDProvider::makeDerivedUUID_forced(nested_node_parent_id, child);

        std::shared_ptr<command::AddVariadicConnector> pass_out =
                std::make_shared<command::AddVariadicConnector>(sub_graph_auuid, ConnectorType::OUTPUT, ci.type);
        executeCommand(pass_out);
        add(pass_out);

        RelayMapping out_map = pass_out->getMap();

        // forwarding connection
        CommandPtr add_internal_connection =
                std::make_shared<command::AddMessageConnection>(sub_graph_auuid, nested_connector_uuid, out_map.internal);
        executeCommand(add_internal_connection);
        add(add_internal_connection);

        // crossing connection
        CommandPtr add_external_connection =
                std::make_shared<command::AddMessageConnection>(parent_auuid, out_map.external, ci.to);
        executeCommand(add_external_connection);
        add(add_external_connection);
    }
}

bool GroupNodes::doUndo()
{
    return Meta::doUndo();
}

bool GroupNodes::doRedo()
{
    locked = false;
    clear();
    return doExecute();
}

