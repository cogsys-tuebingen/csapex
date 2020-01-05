/// HEADER
#include <csapex/core/graphio.h>

/// PROJECT
#include <csapex/info.h>
#include <csapex/model/node.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_facade_impl.h>
#include <csapex/factory/node_factory_impl.h>
#include <csapex/msg/direct_connection.h>
#include <csapex/model/graph_facade_impl.h>
#include <csapex/model/graph/graph_impl.h>
#include <csapex/model/subgraph_node.h>
#include <csapex/model/connection_description.h>
#include <csapex/model/generic_state.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/signal/event.h>
#include <csapex/signal/slot.h>
#include <csapex/model/fulcrum.h>
#include <csapex/utility/assert.h>
#include <csapex/model/node_state.h>
#include <csapex/utility/yaml_node_builder.h>
#include <csapex/serialization/node_serializer.h>
#include <csapex/serialization/snippet.h>
#include <csapex/utility/yaml_io.hpp>
#include <csapex/utility/exceptions.h>
#include <csapex/profiling/profiler.h>
#include <csapex/profiling/timer.h>
#include <csapex/utility/yaml.h>

/// SYSTEM
#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>
#include <sys/types.h>

using namespace csapex;

#define sendNotificationStreamGraphio(args)                                                                                                                                                            \
    {                                                                                                                                                                                                  \
        std::stringstream ss;                                                                                                                                                                          \
        ss << args;                                                                                                                                                                                    \
        sendNotification(ss.str());                                                                                                                                                                    \
    }

GraphIO::GraphIO(GraphFacadeImplementation& graph, NodeFactoryImplementation* node_factory, bool throw_on_error)
  : graph_(graph), node_factory_(node_factory), position_offset_x_(0.0), position_offset_y_(0.0), ignore_forwarding_connections_(false), throw_on_error_(throw_on_error)
{
}

void GraphIO::setIgnoreForwardingConnections(bool ignore)
{
    ignore_forwarding_connections_ = ignore;
}

void GraphIO::saveSettings(YAML::Node& doc)
{
    doc["uuid_map"] = graph_.getLocalGraph()->getUUIDMap();
}

void GraphIO::loadSettings(const YAML::Node& doc)
{
    if (doc["uuid_map"].IsDefined()) {
        graph_.getLocalGraph()->uuids_ = doc["uuid_map"].as<std::map<std::string, int>>();
    }
}

Snippet GraphIO::saveGraph()
{
    YAML::Node yaml;

    saveGraphTo(yaml);

    return Snippet(yaml);
}

void GraphIO::saveGraphTo(YAML::Node& yaml)
{
    TimerPtr timer = getProfiler()->getTimer("save graph");
    timer->restart();

    yaml["version"] = csapex::info::CSAPEX_VERSION;

    saveNodes(yaml);
    saveConnections(yaml);

    {
        auto interlude = timer->step("save view");
        saveViewRequest(graph_, yaml);
    }

    timer->finish();
}

void GraphIO::loadGraph(const Snippet& doc)
{
    YAML::Node yaml;
    doc.toYAML(yaml);

    loadGraphFrom(yaml);
}

void GraphIO::loadGraphFrom(const YAML::Node& doc)
{
    TimerPtr timer = getProfiler()->getTimer("load graph");
    timer->restart();

    SemanticVersion version;
    if(doc["version"].IsDefined()) {
        version = doc["version"].as<SemanticVersion>();
    } else {
        // with 0.9.7 versioning was introduced, so assume that the version was 0.9.6
        version = SemanticVersion(0, 9, 6);
    }

    graph_.getLocalGraph()->beginTransaction();
    {
        auto interlude = timer->step("load nodes");
        loadNodes(doc, version);
    }

    {
        auto interlude = timer->step("load connections");
        loadConnections(doc, version);
    }
    graph_.getLocalGraph()->finalizeTransaction();

    {
        auto interlude = timer->step("load view");
        loadViewRequest(graph_, doc);
    }

    timer->finish();
}

Snippet GraphIO::saveSelectedGraph(const std::vector<UUID>& uuids)
{
    YAML::Node yaml = YAML::Node(YAML::NodeType::Map);

    std::set<UUID> node_set(uuids.begin(), uuids.end());

    std::vector<NodeFacadeImplementationPtr> nodes;
    std::vector<ConnectionDescription> connections;

    for (const UUID& uuid : uuids) {
        NodeFacadePtr nf = graph_.findNodeFacade(uuid);
        NodeFacadeImplementationPtr node = std::dynamic_pointer_cast<NodeFacadeImplementation>(nf);
        apex_assert_hard(node);
        nodes.push_back(node);

        for (const ConnectablePtr& connectable : node->getNodeHandle()->getExternalConnectors()) {
            if (connectable->isOutput()) {
                for (const ConnectionPtr& connection : connectable->getConnections()) {
                    auto target = graph_.getLocalGraph()->findNodeHandleForConnector(connection->to()->getUUID());
                    if (node_set.find(target->getUUID()) != node_set.end()) {
                        connections.push_back(connection->getDescription());
                    }
                }
            }
        }
    }

    saveNodes(yaml, nodes);
    saveConnections(yaml, connections);

    return Snippet(yaml);
}

std::unordered_map<UUID, UUID, UUID::Hasher> GraphIO::loadIntoGraph(const Snippet& snippet, const Point& position, SemanticVersion version)
{
    double min_x = std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();

    YAML::Node blueprint;
    snippet.toYAML(blueprint);

    YAML::Node nodes = blueprint["nodes"];
    if (nodes.IsDefined()) {
        for (std::size_t i = 0, total = nodes.size(); i < total; ++i) {
            const YAML::Node& n = nodes[i];

            std::string type = n["type"].as<std::string>();
            UUID new_uuid = graph_.getLocalGraph()->generateUUID(type);
            UUID blue_print_uuid = UUIDProvider::makeUUID_without_parent(n["uuid"].as<std::string>());

            old_node_uuid_to_new_[blue_print_uuid] = new_uuid;

            if (n["pos"].IsDefined()) {
                double x = n["pos"][0].as<double>();
                double y = n["pos"][1].as<double>();

                if (x < min_x) {
                    min_x = x;
                }
                if (y < min_y) {
                    min_y = y;
                }
            }
        }
    }

    if (min_x != std::numeric_limits<double>::infinity() && min_y != std::numeric_limits<double>::infinity()) {
        position_offset_x_ = position.x - min_x;
        position_offset_y_ = position.y - min_y;
    }

    loadNodes(blueprint, version);
    loadConnections(blueprint, version);

    auto res = old_node_uuid_to_new_;

    old_node_uuid_to_new_.clear();

    position_offset_x_ = 0;
    position_offset_y_ = 0;

    return res;
}

void GraphIO::saveNodes(YAML::Node& yaml)
{
    saveNodes(yaml, graph_.getLocalGraph()->getAllLocalNodeFacades());
}

void GraphIO::saveNodes(YAML::Node& yaml, const std::vector<NodeFacadeImplementationPtr>& nodes)
{
    for (const NodeFacadeImplementationPtr& node : nodes) {
        try {
            YAML::Node yaml_node;
            serializeNode(yaml_node, node);
            yaml["nodes"].push_back(yaml_node);
        } catch (const std::exception& e) {
            sendNotificationStreamGraphio("cannot save state for node " << node->getUUID() << ": " << e.what());
            throw e;
        }
    }
}

void GraphIO::loadNodes(const YAML::Node& doc, SemanticVersion version)
{
    TimerPtr timer = getProfiler()->getTimer("load graph");

    YAML::Node nodes = doc["nodes"];
    if (nodes.IsDefined()) {
        for (std::size_t i = 0, total = nodes.size(); i < total; ++i) {
            const YAML::Node& n = nodes[i];

            auto interlude = timer->step(n["uuid"].as<std::string>());
            loadNode(n, version);
        }
    }
}

UUID GraphIO::readNodeUUID(std::weak_ptr<UUIDProvider> parent, const YAML::Node& doc)
{
    UUID uuid = UUIDProvider::makeUUID_forced(parent, doc.as<std::string>());

    if (!old_node_uuid_to_new_.empty()) {
        auto pos = old_node_uuid_to_new_.find(uuid);
        if (pos != old_node_uuid_to_new_.end()) {
            uuid = old_node_uuid_to_new_[uuid];
        }
    }

    return uuid;
}

UUID GraphIO::readConnectorUUID(std::weak_ptr<UUIDProvider> parent, const YAML::Node& doc)
{
    std::string id = doc.as<std::string>();

    // backward compatibility (trigger instead of event)
    {
        static std::string legacy = UUID::namespace_separator + "trigger_";
        auto pos = id.find(legacy);
        if (pos != id.npos) {
            id = id.substr(0, pos) + UUID::namespace_separator + "event_" + id.substr(pos + legacy.size());
        }
    }

    UUID uuid = UUIDProvider::makeUUID_forced(parent, id);

    if (!old_node_uuid_to_new_.empty()) {
        UUID parent = uuid.parentUUID();

        auto pos = old_node_uuid_to_new_.find(parent);
        if (pos != old_node_uuid_to_new_.end()) {
            parent = old_node_uuid_to_new_[parent];

            uuid = graph_.getLocalGraph()->makeDerivedUUID_forced(parent, uuid.id().getFullName());
        }
    }
    return uuid;
}

void GraphIO::loadNode(const YAML::Node& doc, SemanticVersion version)
{
    UUID uuid = readNodeUUID(graph_.getLocalGraph()->shared_from_this(), doc["uuid"]);

    std::string type = doc["type"].as<std::string>();

    NodeFacadeImplementationPtr node_facade = node_factory_->makeNode(type, uuid, graph_.getLocalGraph());
    if (!node_facade) {
        return;
    }

    try {
        deserializeNode(doc, node_facade, version);

    } catch (const std::exception& e) {
        sendNotificationStreamGraphio("cannot load state for box " << uuid << ": " << type2name(typeid(e)) << ", what=" << e.what());
    }
}

void GraphIO::saveConnections(YAML::Node& yaml)
{
    auto interlude = getProfiler()->getTimer("save graph")->step("save connections");
    saveConnections(yaml, graph_.enumerateAllConnections());
}

void GraphIO::saveConnections(YAML::Node& yaml, const std::vector<ConnectionDescription>& connections)
{
    std::unordered_map<UUID, std::vector<std::pair<UUID, std::string>>, UUID::Hasher> connection_map;

    for (const ConnectionDescription& connection : connections) {
        if (ignore_forwarding_connections_) {
            if (connection.from.type() == "relayout" || connection.to.type() == "relayin" || connection.from.type() == "relayevent" || connection.to.type() == "relayslot") {
                continue;
            }
        }

        std::string type = connection.active ? "active" : "default";

        connection_map[connection.from].push_back(std::make_pair(connection.to, type));

        if (!connection.fulcrums.empty()) {
            YAML::Node fulcrum;
            saveFulcrums(fulcrum, connection);
            yaml["fulcrums"].push_back(fulcrum);
        }
    }

    yaml["connections"] = YAML::Node(YAML::NodeType::Sequence);
    for (const auto& pair : connection_map) {
        YAML::Node entry(YAML::NodeType::Map);
        entry["uuid"] = pair.first.getFullName();
        for (const auto& info : pair.second) {
            entry["targets"].push_back(info.first.getFullName());
            entry["types"].push_back(info.second);
        }
        yaml["connections"].push_back(entry);
    }
}

void GraphIO::loadConnections(const YAML::Node& doc, SemanticVersion version)
{
    if (doc["connections"].IsDefined()) {
        const YAML::Node& connections = doc["connections"];
        apex_assert_hard(connections.Type() == YAML::NodeType::Sequence);

        for (unsigned i = 0; i < connections.size(); ++i) {
            const YAML::Node& connection = connections[i];
            apex_assert_hard(connection.Type() == YAML::NodeType::Map);

            try {
                loadConnection(connection, version);
            } catch (const std::exception& e) {
                sendNotificationStreamGraphio("cannot load connection: " << e.what());
            }
        }
    }

    if (doc["fulcrums"].IsDefined()) {
        const YAML::Node& fulcrums = doc["fulcrums"];
        apex_assert_hard(fulcrums.Type() == YAML::NodeType::Sequence);

        for (unsigned i = 0; i < fulcrums.size(); ++i) {
            const YAML::Node& fulcrum = fulcrums[i];
            apex_assert_hard(fulcrum.Type() == YAML::NodeType::Map);

            try {
                loadFulcrum(fulcrum, version);
            } catch (const std::exception& e) {
                sendNotificationStreamGraphio("cannot load fulcrum: " << e.what());
            }
        }
    }
}

void GraphIO::loadConnection(const YAML::Node& connection, SemanticVersion version)
{
    UUID from_uuid = readConnectorUUID(graph_.getLocalGraph()->shared_from_this(), connection["uuid"]);

    const YAML::Node& targets = connection["targets"];
    apex_assert_hard(targets.Type() == YAML::NodeType::Sequence);

    const YAML::Node& types = connection["types"];
    apex_assert_hard(!types.IsDefined() || (types.Type() == YAML::NodeType::Sequence && targets.size() == types.size()));

    for (unsigned j = 0; j < targets.size(); ++j) {
        UUID to_uuid = readConnectorUUID(graph_.getLocalGraph()->shared_from_this(), targets[j]);

        std::string connection_type;
        if (!types.IsDefined()) {
            connection_type = "default";
        } else {
            connection_type = types[j].as<std::string>();
        }

        ConnectorPtr from = graph_.findConnectorNoThrow(from_uuid);
        if (from) {
            loadConnection(from, to_uuid, connection_type, version);
        } else {
            sendNotificationStreamGraphio("cannot load connection from '" << from_uuid << "' to '" << to_uuid << "', '" << from_uuid << "' doesn't exist.");
        }
    }
}

void GraphIO::saveFulcrums(YAML::Node& fulcrum, const ConnectionDescription& connection)
{
    fulcrum["from"] = connection.from.getFullName();
    fulcrum["to"] = connection.to.getFullName();

    for (const Fulcrum& f : connection.fulcrums) {
        YAML::Node pt;
        pt.push_back(f.pos().x);
        pt.push_back(f.pos().y);

        fulcrum["pts"].push_back(pt);
    }

    for (const Fulcrum& f : connection.fulcrums) {
        YAML::Node handle;
        handle.push_back(f.handleIn().x);
        handle.push_back(f.handleIn().y);
        handle.push_back(f.handleOut().x);
        handle.push_back(f.handleOut().y);

        fulcrum["handles"].push_back(handle);
    }

    for (const Fulcrum& f : connection.fulcrums) {
        fulcrum["types"].push_back(f.type());
    }
}

void GraphIO::loadFulcrum(const YAML::Node& fulcrum, SemanticVersion version)
{
    YAML::Node from_node = fulcrum["from"];
    if (!from_node.IsDefined()) {
        return;
    }

    YAML::Node to_node = fulcrum["to"];
    if (!to_node.IsDefined()) {
        return;
    }

    std::string from_uuid_tmp = from_node.as<std::string>();
    std::string to_uuid_tmp = to_node.as<std::string>();

    UUID from_uuid = UUIDProvider::makeUUID_forced(graph_.getLocalGraph()->shared_from_this(), from_uuid_tmp);
    UUID to_uuid = UUIDProvider::makeUUID_forced(graph_.getLocalGraph()->shared_from_this(), to_uuid_tmp);

    ConnectorPtr from = graph_.findConnector(from_uuid);
    if (from == nullptr) {
        sendNotificationStreamGraphio("cannot load fulcrum, connector with uuid '" << from_uuid << "' doesn't exist.");
        return;
    }

    ConnectorPtr to = graph_.findConnector(to_uuid);
    if (to == nullptr) {
        sendNotificationStreamGraphio("cannot load fulcrum, connector with uuid '" << to_uuid << "' doesn't exist.");
        return;
    }

    ConnectionPtr connection = graph_.getLocalGraph()->getConnection(from->getUUID(), to->getUUID());

    std::vector<std::vector<double>> pts = fulcrum["pts"].as<std::vector<std::vector<double>>>();

    std::vector<std::vector<double>> handles;
    bool has_handle = fulcrum["handles"].IsDefined();
    if (has_handle) {
        handles = fulcrum["handles"].as<std::vector<std::vector<double>>>();
    }

    std::vector<int> types;
    if (fulcrum["types"].IsDefined()) {
        types = fulcrum["types"].as<std::vector<int>>();
    }

    int n = pts.size();
    for (int i = 0; i < n; ++i) {
        int type = (!types.empty()) ? types[i] : Fulcrum::FULCRUM_LINEAR;
        if (has_handle) {
            Point in(handles[i][0], handles[i][1]);
            Point out(handles[i][2], handles[i][3]);
            connection->addFulcrum(i, Point(pts[i][0], pts[i][1]), type, in, out);
        } else {
            connection->addFulcrum(i, Point(pts[i][0], pts[i][1]), type);
        }
    }
}

void GraphIO::loadConnection(ConnectorPtr from, const UUID& to_uuid, const std::string& connection_type, SemanticVersion version)
{
    try {
        NodeHandle* target = graph_.getLocalGraph()->findNodeHandleForConnector(to_uuid);

        InputPtr in = std::dynamic_pointer_cast<Input>(target->getConnector(to_uuid));
        if (!in) {
            sendNotificationStreamGraphio("cannot load message connection from " << from->getUUID() << " to " << to_uuid << ", input doesn't exist.");
            return;
        }

        OutputPtr out = std::dynamic_pointer_cast<Output>(from);

        if (out && in) {
            // TODO: make connection factory
            ConnectionPtr c = DirectConnection::connect(out, in);
            if (connection_type == "active") {
                c->setActive(true);
            }
            graph_.getLocalGraph()->addConnection(c);
        }

    } catch (const std::exception& e) {
        sendNotificationStreamGraphio("cannot load connection: " << e.what());
    } catch (const Failure& e) {
        sendNotificationStreamGraphio("failure loading connection: " << e.what());
    }
}

void GraphIO::serializeNode(YAML::Node& doc, NodeFacadeImplementationConstPtr node_facade)
{
    auto interlude = getProfiler()->getTimer("save graph")->step("serialize node");

    node_facade->getNodeState()->writeYaml(doc);

    auto node = node_facade->getNode();
    if (node) {
        // hook for nodes to serialize
        NodeSerializer::instance().serialize(*node, doc);

        if (node_facade->isGraph()) {
            GraphFacadeImplementationPtr subgraph = graph_.getLocalSubGraph(node_facade->getUUID());
            if (subgraph) {
                YAML::Node subgraph_yaml;
                GraphIO sub_graph_io(*subgraph, node_factory_);
                slim_signal::ScopedConnection connection = sub_graph_io.saveViewRequest.connect(saveViewRequest);

                sub_graph_io.saveGraphTo(subgraph_yaml);
                doc["subgraph"] = subgraph_yaml;
            }
        }
    }
}

void GraphIO::deserializeNode(const YAML::Node& doc, NodeFacadeImplementationPtr node_facade, SemanticVersion version)
{
    NodeState::Ptr s = node_facade->getNodeState();
    s->readYaml(doc);
    s->getParameterState()->initializePersistentParameters();

    int x = doc["pos"][0].as<double>() + position_offset_x_;
    int y = doc["pos"][1].as<double>() + position_offset_y_;

    if (x != 0 || y != 0) {
        s->setPos(Point(x, y));
    }

    // hook for nodes to deserialize
    auto node = node_facade->getNode();
    apex_assert_hard(node);

    NodeSerializer::instance().deserialize(*node, doc);

    graph_.getLocalGraph()->addNode(node_facade);

    node_facade->handleChangedParameters();

    if (node_facade->isGraph()) {
        GraphFacadeImplementationPtr subgraph = graph_.getLocalSubGraph(node_facade->getUUID());
        if (subgraph) {
            GraphIO sub_graph_io(*subgraph, node_factory_, throw_on_error_);
            slim_signal::ScopedConnection connection = sub_graph_io.loadViewRequest.connect(loadViewRequest);

            sub_graph_io.loadGraph(doc["subgraph"]);
        }
    }
}

void GraphIO::sendNotification(const std::string& notification)
{
    if (throw_on_error_) {
        throw std::logic_error(notification);
    } else {
        graph_.getLocalGraph()->notification(Notification(graph_.getLocalGraph()->getAbsoluteUUID(), notification));
    }
}
