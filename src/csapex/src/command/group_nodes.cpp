/// HEADER
#include <csapex/command/group_nodes.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/model/subgraph_node.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_state.h>
#include <csapex/command/delete_node.h>
#include <csapex/command/command_factory.h>
#include <csapex/command/add_node.h>
#include <csapex/model/graph_facade.h>
#include <csapex/scheduling/thread_pool.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/model/connection.h>
#include <csapex/core/graphio.h>
#include <csapex/command/add_variadic_connector.h>
#include <csapex/command/add_connection.h>
#include <csapex/utility/assert.h>

/// SYSTEM
#include <sstream>

using namespace csapex;
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
    SubgraphNode* graph = getSubgraphNode();
    {
        GraphIO io(graph, getNodeFactory());
        io.setIgnoreForwardingConnections(true);
        serialized_snippet_ = io.saveSelectedGraph(uuids);
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

    CommandFactory cf(graph_facade_);

    CommandPtr del = cf.deleteAllNodes(uuids);
    executeCommand(del);
    add(del);

//    for(NodeHandle* nh : nodes) {
//        UUID old_uuid = nh->getUUID();
//        CommandPtr del = std::make_shared<command::DeleteNode>(getGraphFacade()->getAbsoluteUUID(), old_uuid);
//        executeCommand(del);
//        add(del);
//    }

    pasteSelection(sub_graph_auuid);

    mapConnections(parent_auuid, sub_graph_auuid);

    return true;
}

void GroupNodes::findNodes(SubgraphNode* graph)
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
    mapMessageGoingIn(parent_auuid, sub_graph_auuid);
    mapMessageGoingOut(parent_auuid, sub_graph_auuid);
    mapSignalGoingIn(parent_auuid, sub_graph_auuid);
    mapSignalGoingOut(parent_auuid, sub_graph_auuid);
}

void GroupNodes::mapMessageGoingIn(AUUID parent_auuid, AUUID sub_graph_auuid)
{
    std::unordered_map<UUID, RelayMapping, UUID::Hasher> cache;
    for(const ConnectionInformation& ci : connections_going_in) {
        UUID nested_node_parent_id = old_uuid_to_new.at(ci.to.parentUUID());
        std::string child = ci.to.id().getFullName();
        UUID nested_connector_uuid = UUIDProvider::makeDerivedUUID_forced(nested_node_parent_id, child);

        RelayMapping in_map;
        if(cache.find(ci.to) != cache.end()) {
            in_map = cache.at(ci.to);

        } else {
            std::shared_ptr<command::AddVariadicConnector> pass_out =
                    std::make_shared<command::AddVariadicConnector>(parent_auuid, sub_graph_auuid, ConnectorType::INPUT, ci.type, ci.to_label);
            executeCommand(pass_out);
            add(pass_out);
            in_map = pass_out->getMap();
            cache[ci.to] = in_map;

            // forwarding connection
            CommandPtr add_internal_connection =
                    std::make_shared<command::AddConnection>(sub_graph_auuid, in_map.internal, nested_connector_uuid, ci.active);
            executeCommand(add_internal_connection);
            add(add_internal_connection);

        }


        // crossing connection
        CommandPtr add_external_connection =
                std::make_shared<command::AddConnection>(parent_auuid, ci.from, in_map.external, ci.active);
        executeCommand(add_external_connection);
        add(add_external_connection);
    }
}

void GroupNodes::mapMessageGoingOut(AUUID parent_auuid, AUUID sub_graph_auuid)
{
    std::unordered_map<UUID, RelayMapping, UUID::Hasher> cache;
    for(const ConnectionInformation& ci : connections_going_out) {
        UUID nested_node_parent_id = old_uuid_to_new.at(ci.from.parentUUID());
        std::string child = ci.from.id().getFullName();
        UUID nested_connector_uuid = UUIDProvider::makeDerivedUUID_forced(nested_node_parent_id, child);

        RelayMapping out_map;
        if(cache.find(ci.from) != cache.end()) {
            out_map = cache.at(ci.from);

        } else {
            std::shared_ptr<command::AddVariadicConnector> pass_out =
                    std::make_shared<command::AddVariadicConnector>(parent_auuid, sub_graph_auuid, ConnectorType::OUTPUT, ci.type, ci.from_label);
            executeCommand(pass_out);
            add(pass_out);
            out_map = pass_out->getMap();
            cache[ci.from] = out_map;

            // forwarding connection
            CommandPtr add_internal_connection =
                    std::make_shared<command::AddConnection>(sub_graph_auuid, nested_connector_uuid, out_map.internal, ci.active);
            executeCommand(add_internal_connection);
            add(add_internal_connection);

        }

        // crossing connection
        CommandPtr add_external_connection =
                std::make_shared<command::AddConnection>(parent_auuid, out_map.external, ci.to, ci.active);
        executeCommand(add_external_connection);
        add(add_external_connection);
    }
}

void GroupNodes::mapSignalGoingIn(AUUID parent_auuid, AUUID sub_graph_auuid)
{
    std::unordered_map<UUID, RelayMapping, UUID::Hasher> cache;
    for(const ConnectionInformation& ci : signals_going_in) {
        UUID nested_node_parent_id = old_uuid_to_new.at(ci.to.parentUUID());
        std::string child = ci.to.id().getFullName();
        UUID nested_connector_uuid = UUIDProvider::makeDerivedUUID_forced(nested_node_parent_id, child);

        RelayMapping in_map;
        if(cache.find(ci.to) != cache.end()) {
            in_map = cache.at(ci.to);

        } else {
            std::shared_ptr<command::AddVariadicConnector> pass_out =
                    std::make_shared<command::AddVariadicConnector>(parent_auuid, sub_graph_auuid, ConnectorType::SLOT_T, ci.type, ci.to_label);
            executeCommand(pass_out);
            add(pass_out);
            in_map = pass_out->getMap();
            cache[ci.to] = in_map;

            // forwarding connection
            CommandPtr add_internal_connection =
                    std::make_shared<command::AddConnection>(sub_graph_auuid, in_map.internal, nested_connector_uuid, ci.active);
            executeCommand(add_internal_connection);
            add(add_internal_connection);
        }

        // crossing connection
        CommandPtr add_external_connection =
                std::make_shared<command::AddConnection>(parent_auuid, ci.from, in_map.external, ci.active);
        executeCommand(add_external_connection);
        add(add_external_connection);
    }
}

void GroupNodes::mapSignalGoingOut(AUUID parent_auuid, AUUID sub_graph_auuid)
{
    std::unordered_map<UUID, RelayMapping, UUID::Hasher> cache;
    for(const ConnectionInformation& ci : signals_going_out) {
        UUID nested_node_parent_id = old_uuid_to_new.at(ci.from.parentUUID());
        std::string child = ci.from.id().getFullName();
        UUID nested_connector_uuid = UUIDProvider::makeDerivedUUID_forced(nested_node_parent_id, child);


        RelayMapping out_map;
        if(cache.find(ci.from) != cache.end()) {
            out_map = cache.at(ci.from);

        } else {
            std::shared_ptr<command::AddVariadicConnector> pass_out =
                    std::make_shared<command::AddVariadicConnector>(parent_auuid, sub_graph_auuid, ConnectorType::EVENT, ci.type, ci.from_label);
            executeCommand(pass_out);
            add(pass_out);
            out_map = pass_out->getMap();
            cache[ci.from] = out_map;

            // forwarding connection
            CommandPtr add_internal_connection =
                    std::make_shared<command::AddConnection>(sub_graph_auuid, nested_connector_uuid, out_map.internal, ci.active);
            executeCommand(add_internal_connection);
            add(add_internal_connection);
        }

        // crossing connection
        CommandPtr add_external_connection =
                std::make_shared<command::AddConnection>(parent_auuid, out_map.external, ci.to, ci.active);
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

