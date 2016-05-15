/// HEADER
#include <csapex/core/graphio.h>

/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_worker.h>
#include <csapex/factory/node_factory.h>
#include <csapex/msg/bundled_connection.h>
#include <csapex/model/graph_facade.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/signal/event.h>
#include <csapex/signal/slot.h>
#include <csapex/model/fulcrum.h>
#include <csapex/utility/assert.h>
#include <csapex/model/node_state.h>
#include <csapex/utility/yaml_node_builder.h>
#include <csapex/serialization/serialization.h>
#include <csapex/utility/yaml_io.hpp>
#include <csapex/utility/exceptions.h>

/// SYSTEM
#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <sys/types.h>

using namespace csapex;

GraphIO::GraphIO(Graph *graph, NodeFactory* node_factory)
    : graph_(graph), node_factory_(node_factory),
      position_offset_x_(0.0), position_offset_y_(0.0),

      ignore_forwarding_connections_(false)
{
}


void GraphIO::setIgnoreForwardingConnections(bool ignore)
{
    ignore_forwarding_connections_ = ignore;
}

void GraphIO::saveSettings(YAML::Node& doc)
{
    doc["uuid_map"] = graph_->getUUIDMap();
}

void GraphIO::loadSettings(const YAML::Node &doc)
{
    if(doc["uuid_map"].IsDefined()) {
        graph_->uuids_ = doc["uuid_map"].as<std::map<std::string, int> >();
    }
}

void GraphIO::saveGraph(YAML::Node &yaml)
{
    saveNodes(yaml);
    saveConnections(yaml);

    saveViewRequest(graph_, yaml);
}

void GraphIO::loadGraph(const YAML::Node& doc)
{
    loadNodes(doc);
    loadConnections(doc);

    loadViewRequest(graph_, doc);
}


void GraphIO::saveSelectedGraph(YAML::Node &yaml, const std::vector<UUID> &uuids)
{
    std::set<UUID> node_set(uuids.begin(), uuids.end());

    std::vector<NodeHandle*> nodes;
    std::vector<ConnectionPtr> connections;

    for(const UUID& uuid : uuids) {
        NodeHandle* node = graph_->findNodeHandle(uuid);
        nodes.push_back(node);

        for(const ConnectablePtr& connectable : node->getExternalConnectors()) {
            if(connectable->isOutput()) {
                for(const ConnectionPtr& connection : connectable->getConnections()) {
                    auto target = graph_->findNodeHandleForConnector(connection->to()->getUUID());
                    if(node_set.find(target->getUUID()) != node_set.end()) {
                        connections.push_back(connection);
                    }
                }
            }
        }
    }

    saveNodes(yaml, nodes);
    saveConnections(yaml, connections);
}

std::unordered_map<UUID, UUID, UUID::Hasher>
GraphIO::loadIntoGraph(const YAML::Node &blueprint, const Point& position)
{
    double min_x = std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();

    YAML::Node nodes = blueprint["nodes"];
    if(nodes.IsDefined()) {
        for(std::size_t i = 0, total = nodes.size(); i < total; ++i) {
            const YAML::Node& n = nodes[i];

            std::string type = n["type"].as<std::string>();
            UUID new_uuid = graph_->generateUUID(type);
            UUID blue_print_uuid = UUIDProvider::makeUUID_without_parent(n["uuid"].as<std::string>());

            old_node_uuid_to_new_[blue_print_uuid] = new_uuid;

            if(n["pos"].IsDefined()) {
                double x = n["pos"][0].as<double>();
                double y = n["pos"][1].as<double>();

                if(x < min_x) {
                    min_x = x;
                }
                if(y < min_y) {
                    min_y = y;
                }
            }
        }
    }

    if(min_x != std::numeric_limits<double>::infinity() &&
            min_y != std::numeric_limits<double>::infinity()) {
        position_offset_x_ = position.x - min_x;
        position_offset_y_ = position.y - min_y;
    }

    loadNodes(blueprint);
    loadConnections(blueprint);

    auto res = old_node_uuid_to_new_;

    old_node_uuid_to_new_.clear();

    position_offset_x_ = 0;
    position_offset_y_ = 0;

    return res;
}

void GraphIO::saveNodes(YAML::Node &yaml)
{
    saveNodes(yaml, graph_->getAllNodeHandles());
}

void GraphIO::saveNodes(YAML::Node &yaml, const std::vector<NodeHandle*>& nodes)
{
    for(NodeHandle* node : nodes) {
        try {
            YAML::Node yaml_node;
            serializeNode(yaml_node, node);
            yaml["nodes"].push_back(yaml_node);
        } catch(const std::exception& e) {
            std::cerr << "cannot save state for node " << node->getUUID() << ": " << e.what() << std::endl;
            throw e;
        }
    }
}

void GraphIO::loadNodes(const YAML::Node& doc)
{
    YAML::Node nodes = doc["nodes"];
    if(nodes.IsDefined()) {
        for(std::size_t i = 0, total = nodes.size(); i < total; ++i) {
            const YAML::Node& n = nodes[i];
            loadNode(n);
        }
    }
}

UUID GraphIO::readNodeUUID(std::weak_ptr<UUIDProvider> parent, const YAML::Node& doc)
{
    UUID uuid = UUIDProvider::makeUUID_forced(parent, doc.as<std::string>());


    if(!old_node_uuid_to_new_.empty()) {
        auto pos = old_node_uuid_to_new_.find(uuid);
        if(pos != old_node_uuid_to_new_.end()) {
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
        if(pos != id.npos) {
            id = id.substr(0, pos) + UUID::namespace_separator + "event_" + id.substr(pos + legacy.size());
        }
    }

    UUID uuid = UUIDProvider::makeUUID_forced(parent, id);

    if(!old_node_uuid_to_new_.empty()) {
        UUID parent = uuid.parentUUID();

        auto pos = old_node_uuid_to_new_.find(parent);
        if(pos != old_node_uuid_to_new_.end()) {
            parent = old_node_uuid_to_new_[parent];

            uuid = graph_->makeDerivedUUID_forced(parent, uuid.id());
        }
    }
    return uuid;
}

void GraphIO::loadNode(const YAML::Node& doc)
{
    UUID uuid = readNodeUUID(graph_->shared_from_this(), doc["uuid"]);

    std::string type = doc["type"].as<std::string>();

    NodeHandlePtr node_handle = node_factory_->makeNode(type, uuid, graph_);
    if(!node_handle) {
        return;
    }

    try {
        deserializeNode(doc, node_handle);

    } catch(const std::exception& e) {
        std::cerr << "cannot load state for box " << uuid << ": " << type2name(typeid(e)) << ", what=" << e.what() << std::endl;
    }
}

void GraphIO::saveConnections(YAML::Node &yaml)
{
    saveConnections(yaml, graph_->getConnections());
}

void GraphIO::saveConnections(YAML::Node &yaml, const std::vector<ConnectionPtr>& connections)
{
    std::unordered_map<UUID, std::vector<std::pair<UUID, std::string>>, UUID::Hasher> connection_map;

    for(const ConnectionPtr& connection : connections) {
        if(ignore_forwarding_connections_) {
            if(connection->from()->getUUID().type() == "relayout" ||
                    connection->to()->getUUID().type() == "relayin" ||
                    connection->from()->getUUID().type() == "relaytrigger" ||
                    connection->to()->getUUID().type() == "relayslot") {
                continue;
            }
        }

        std::string type = connection->isActive() ? "active" : "default";

        connection_map[connection->from()->getUUID()].push_back(std::make_pair(connection->to()->getUUID(), type));

        if(connection->getFulcrumCount() > 0) {
            YAML::Node fulcrum;
            saveFulcrums(fulcrum, connection.get());
            yaml["fulcrums"].push_back(fulcrum);
        }
    }

    yaml["connections"] = YAML::Node(YAML::NodeType::Sequence);
    for(const auto& pair : connection_map) {
        YAML::Node entry(YAML::NodeType::Map);
        entry["uuid"] = pair.first.getFullName();
        for(const auto& info : pair.second) {
            entry["targets"].push_back(info.first.getFullName());
            entry["types"].push_back(info.second);
        }
        yaml["connections"].push_back(entry);
    }
}

void GraphIO::loadConnections(const YAML::Node &doc)
{
    if(doc["connections"].IsDefined()) {
        const YAML::Node& connections = doc["connections"];
        apex_assert_hard(connections.Type() == YAML::NodeType::Sequence);

        for(unsigned i = 0; i < connections.size(); ++i) {
            const YAML::Node& connection = connections[i];
            apex_assert_hard(connection.Type() == YAML::NodeType::Map);

            try {
                loadConnection(connection);
            } catch(const std::exception& e) {
                std::cerr << "cannot load connection: " << e.what() << std::endl;
            }
        }
    }

    if(doc["fulcrums"].IsDefined()) {
        const YAML::Node& fulcrums = doc["fulcrums"];
        apex_assert_hard(fulcrums.Type() == YAML::NodeType::Sequence);


        for(unsigned i = 0; i < fulcrums.size(); ++i) {
            const YAML::Node& fulcrum = fulcrums[i];
            apex_assert_hard(fulcrum.Type() == YAML::NodeType::Map);

            try {
                loadFulcrum(fulcrum);
            } catch(const std::exception& e) {
                std::cerr << "cannot load fulcrum: " << e.what() << std::endl;
            }
        }
    }
}

void GraphIO::loadConnection(const YAML::Node& connection)
{
    UUID from_uuid =  readConnectorUUID(graph_->shared_from_this(), connection["uuid"]);

    const YAML::Node& targets = connection["targets"];
    apex_assert_hard(targets.Type() == YAML::NodeType::Sequence);

    const YAML::Node& types = connection["types"];
    apex_assert_hard(!types.IsDefined() || (types.Type() == YAML::NodeType::Sequence && targets.size() == types.size()));

    for(unsigned j=0; j<targets.size(); ++j) {
        UUID to_uuid = readConnectorUUID(graph_->shared_from_this(), targets[j]);

        std::string connection_type;
        if(!types.IsDefined()) {
            connection_type = "default";
        } else {
            connection_type = types[j].as<std::string>();
        }

        std::string type = to_uuid.type();

        Connectable* from = graph_->findConnector(from_uuid);

        if(type == "in" || type == "relayin") {
            loadMessageConnection(from, to_uuid, connection_type);
        } else {
            loadSignalConnection(from, to_uuid, connection_type);
        }
    }
}

void GraphIO::saveFulcrums(YAML::Node &fulcrum, const Connection *connection)
{
    fulcrum["from"] = connection->from()->getUUID().getFullName();
    fulcrum["to"] = connection->to()->getUUID().getFullName();

    for(const Fulcrum::Ptr& f : connection->getFulcrums()) {
        YAML::Node pt;
        pt.push_back(f->pos().x);
        pt.push_back(f->pos().y);

        fulcrum["pts"].push_back(pt);
    }

    for(const Fulcrum::Ptr& f : connection->getFulcrums()) {
        YAML::Node handle;
        handle.push_back(f->handleIn().x);
        handle.push_back(f->handleIn().y);
        handle.push_back(f->handleOut().x);
        handle.push_back(f->handleOut().y);

        fulcrum["handles"].push_back(handle);
    }

    for(const Fulcrum::Ptr& f : connection->getFulcrums()) {
        fulcrum["types"].push_back(f->type());
    }

}

void GraphIO::loadFulcrum(const YAML::Node& fulcrum)
{
    std::string from_uuid_tmp = fulcrum["from"].as<std::string>();
    std::string to_uuid_tmp = fulcrum["to"].as<std::string>();

    UUID from_uuid = UUIDProvider::makeUUID_forced(graph_->shared_from_this(), from_uuid_tmp);
    UUID to_uuid = UUIDProvider::makeUUID_forced(graph_->shared_from_this(), to_uuid_tmp);

    Connectable* from = graph_->findConnector(from_uuid);
    if(from == nullptr) {
        std::cerr << "cannot load fulcrum, connector with uuid '" << from_uuid << "' doesn't exist." << std::endl;
        return;
    }

    Connectable* to = graph_->findConnector(to_uuid);
    if(to == nullptr) {
        std::cerr << "cannot load fulcrum, connector with uuid '" << to_uuid << "' doesn't exist." << std::endl;
        return;
    }

    ConnectionPtr connection = graph_->getConnection(from, to);

    std::vector< std::vector<double> > pts = fulcrum["pts"].as<std::vector< std::vector<double> > >();

    std::vector< std::vector<double> > handles;
    bool has_handle = fulcrum["handles"].IsDefined();
    if(has_handle) {
        handles = fulcrum["handles"].as<std::vector< std::vector<double> > >();
    }

    std::vector<int> types;
    if(fulcrum["types"].IsDefined()) {
        types = fulcrum["types"].as<std::vector<int> >();
    }

    int n = pts.size();
    for(int i = 0; i < n; ++i) {
        int type = (!types.empty()) ? types[i] : Fulcrum::CURVE;
        if(has_handle) {
            Point in(handles[i][0], handles[i][1]);
            Point out(handles[i][2], handles[i][3]);
            connection->addFulcrum(i, Point(pts[i][0], pts[i][1]), type, in, out);
        } else {
            connection->addFulcrum(i, Point(pts[i][0], pts[i][1]), type);
        }
    }
}

void GraphIO::loadMessageConnection(Connectable* from, const UUID& to_uuid, const std::string& connection_type)
{
    try {
        NodeHandle* target = graph_->findNodeHandleForConnector(to_uuid);

        Input* in = target->getInput(to_uuid);
        if(!in) {
            std::cerr << "cannot load message connection from " << from->getUUID() << " to " << to_uuid << ", input doesn't exist." << std::endl;
            return;
        }

        Output* out = dynamic_cast<Output*>(from);
        if(out && in) {
            ConnectionPtr c = BundledConnection::connect(out, in);
            if(connection_type == "active") {
                c->setActive(true);
            }
            graph_->addConnection(c);
        }

    } catch(const std::exception& e) {
        std::cerr << "cannot load connection: " << e.what() << std::endl;
    }
}


void GraphIO::loadSignalConnection(Connectable* from, const UUID& to_uuid, const std::string& connection_type)
{
    try {
        NodeHandle* target = graph_->findNodeHandleForConnector(to_uuid);

        Slot* in = target->getSlot(to_uuid);
        if(!in) {
            std::cerr << "cannot load message connection from " << from->getUUID() << " to " << to_uuid << ", slot doesn't exist." << std::endl;
            return;
        }

        Event* out = dynamic_cast<Event*>(from);
        if(out && in) {
            // TODO: make connection factory
            ConnectionPtr c = DirectConnection::connect(out, in);
            if(connection_type == "active") {
                c->setActive(true);
            }
            graph_->addConnection(c);
        }

    } catch(const std::exception& e) {
        std::cerr << "cannot load connection: " << e.what() << std::endl;
    } catch(const Failure& e) {
        std::cerr << "failure loading connection: " << e.what() << std::endl;
    }
}


void GraphIO::serializeNode(YAML::Node& doc, NodeHandle* node_handle)
{
    node_handle->getNodeState()->writeYaml(doc);

    auto node = node_handle->getNode().lock();
    if(node) {
        // hook for nodes to serialize
        Serialization::instance().serialize(*node, doc);

        GraphPtr subgraph = std::dynamic_pointer_cast<Graph>(node);
        if(subgraph) {
            YAML::Node subgraph_yaml;
            GraphIO sub_graph_io(subgraph.get(), node_factory_);
            sub_graph_io.saveGraph(subgraph_yaml);
            doc["subgraph"] = subgraph_yaml;
        }
    }
}

void GraphIO::deserializeNode(const YAML::Node& doc, NodeHandlePtr node_handle)
{

    NodeState::Ptr s = node_handle->getNodeStateCopy();
    s->readYaml(doc);

    int x = doc["pos"][0].as<double>() + position_offset_x_;
    int y = doc["pos"][1].as<double>() + position_offset_y_;

    if(x != 0 || y != 0) {
        s->setPos(Point(x,y));
    }
    node_handle->setNodeState(s);

    // hook for nodes to deserialize
    auto node = node_handle->getNode().lock();
    apex_assert_hard(node);

    Serialization::instance().deserialize(*node, doc);

    graph_->addNode(node_handle);

    GraphPtr subgraph = std::dynamic_pointer_cast<Graph>(node);
    if(subgraph) {
        GraphIO sub_graph_io(subgraph.get(), node_factory_);
        slim_signal::ScopedConnection connection = sub_graph_io.loadViewRequest.connect(loadViewRequest);

        sub_graph_io.loadGraph(doc["subgraph"]);
    }
}
