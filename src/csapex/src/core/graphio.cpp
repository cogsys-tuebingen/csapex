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
#include <csapex/signal/trigger.h>
#include <csapex/signal/slot.h>
#include <csapex/model/fulcrum.h>
#include <csapex/utility/assert.h>
#include <csapex/model/node_state.h>
#include <csapex/utility/yaml_node_builder.h>
#include <csapex/serialization/serialization.h>
#include <csapex/signal/signal_connection.h>
#include <csapex/utility/yaml_io.hpp>

/// SYSTEM
#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <sys/types.h>

using namespace csapex;

GraphIO::GraphIO(Graph *graph, NodeFactory* node_factory)
    : graph_(graph), node_factory_(node_factory),
      position_offset_x_(0.0), position_offset_y_(0.0)
{
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

        for(const ConnectablePtr& connectable : node->getAllConnectors()) {
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
            UUID blue_print_uuid = n["uuid"].as<UUID>();

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

    // save forwarded inputs
    YAML::Node fw_in(YAML::NodeType::Sequence);
    for(const auto& pair : graph_->forward_inputs_) {
        Input* i = pair.first;

        YAML::Node in(YAML::NodeType::Map);
        in["uuid_external"] = i->getUUID();
        in["uuid_internal"] = pair.second->getUUID();
        in["type"] = i->getType()->typeName();
        in["optional"] = i->isOptional();
        in["label"] = i->getLabel();

        fw_in.push_back(in);
    }
    yaml["forward_in"] = fw_in;

    // save forwarded outputs
    YAML::Node fw_out(YAML::NodeType::Sequence);
    for(const auto& pair : graph_->forward_outputs_) {
        Output* o = pair.first;

        YAML::Node out(YAML::NodeType::Map);
        out["uuid_external"] = o->getUUID();
        out["uuid_internal"] = pair.second->getUUID();
        out["type"] = o->getType()->typeName();
        out["label"] = o->getLabel();

        fw_out.push_back(out);
    }
    yaml["forward_out"] = fw_out;
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

    YAML::Node fw_in = doc["forward_in"];
    if(fw_in.IsDefined()) {
        for(std::size_t i = 0, total = fw_in.size(); i < total; ++i) {
            YAML::Node node = fw_in[i];
            ConnectionTypeConstPtr type = MessageFactory::createMessage(node["type"].as<std::string>());
            graph_->addForwardingInput(node["uuid_internal"].as<UUID>(), node["uuid_external"].as<UUID>(),
                    type, node["label"].as<std::string>(), node["optional"].as<bool>());
        }
    }
    YAML::Node fw_out = doc["forward_out"];
    if(fw_out.IsDefined()) {
        for(std::size_t i = 0, total = fw_out.size(); i < total; ++i) {
            YAML::Node node = fw_out[i];
            ConnectionTypeConstPtr type = MessageFactory::createMessage(node["type"].as<std::string>());
            graph_->addForwardingOutput(node["uuid_internal"].as<UUID>(), node["uuid_external"].as<UUID>(),
                    type, node["label"].as<std::string>());
        }
    }
}

UUID GraphIO::readNodeUUID(const YAML::Node& doc)
{
    UUID uuid = doc.as<UUID>();

    if(!old_node_uuid_to_new_.empty()) {
        auto pos = old_node_uuid_to_new_.find(uuid);
        if(pos != old_node_uuid_to_new_.end()) {
            uuid = old_node_uuid_to_new_[uuid];
        }
    }

    return uuid;
}

UUID GraphIO::readConnectorUUID(const YAML::Node& doc)
{
    UUID uuid = doc.as<UUID>();

    if(!old_node_uuid_to_new_.empty()) {
        UUID parent = uuid.parentUUID();

        auto pos = old_node_uuid_to_new_.find(parent);
        if(pos != old_node_uuid_to_new_.end()) {
            parent = old_node_uuid_to_new_[parent];

            uuid = graph_->makeDerivedUUID(parent, uuid.id());
        }
    }
    return uuid;
}

void GraphIO::loadNode(const YAML::Node& doc)
{
    UUID uuid = readNodeUUID(doc["uuid"]);

    std::string type = doc["type"].as<std::string>();

    NodeHandlePtr node_handle = node_factory_->makeNode(type, uuid, graph_);
    if(!node_handle) {
        return;
    }

    try {
        deserializeNode(doc, node_handle);

    } catch(const std::exception& e) {
        std::cerr << "cannot load state for box " << uuid << ": " << typeid(e).name() << ", what=" << e.what() << std::endl;
    }
}

void GraphIO::saveConnections(YAML::Node &yaml)
{
    saveConnections(yaml, graph_->getConnections());
}

void GraphIO::saveConnections(YAML::Node &yaml, const std::vector<ConnectionPtr>& connections)
{
    std::unordered_map<UUID, std::vector<UUID>, UUID::Hasher> connection_map;

    for(const ConnectionPtr& connection : connections) {
        connection_map[connection->from()->getUUID()].push_back(connection->to()->getUUID());

        if(connection->getFulcrumCount() > 0) {
            YAML::Node fulcrum;
            saveFulcrums(fulcrum, connection.get());
            yaml["fulcrums"].push_back(fulcrum);
        }
    }

    yaml["connections"] = YAML::Node(YAML::NodeType::Sequence);
    for(const auto& pair : connection_map) {
        YAML::Node entry(YAML::NodeType::Map);
        entry["uuid"] = pair.first;
        for(const auto& uuid : pair.second) {
            entry["targets"].push_back(uuid);
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
    UUID from_uuid =  readConnectorUUID(connection["uuid"]);

    NodeHandle* parent = nullptr;
    try {
        parent = graph_->findNodeHandleForConnector(from_uuid);

    } catch(const std::exception& e) {
        std::cerr << "cannot find connector '" << from_uuid << "'" << std::endl;
        return;
    }

    if(!parent) {
        std::cerr << "cannot find connector '" << from_uuid << "'" << std::endl;
        return;
    }

    const YAML::Node& targets = connection["targets"];
    apex_assert_hard(targets.Type() == YAML::NodeType::Sequence);

    for(unsigned j=0; j<targets.size(); ++j) {
        UUID to_uuid = readConnectorUUID(targets[j]);

        Connectable* from = parent->getOutput(from_uuid);
        if(from != nullptr) {
            loadMessageConnection(from, parent, to_uuid);
            continue;

        } else {
            from = parent->getTrigger(from_uuid);
            if(from != nullptr) {
                loadSignalConnection(from, to_uuid);
                continue;
            }
        }

        std::cerr << "cannot load connection, connector with uuid '" << from_uuid << "' doesn't exist." << std::endl;

    }
}

void GraphIO::saveFulcrums(YAML::Node &fulcrum, const Connection *connection)
{
    fulcrum["from"] = connection->from()->getUUID();
    fulcrum["to"] = connection->to()->getUUID();

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

    UUID from_uuid = graph_->makeUUID(from_uuid_tmp);
    UUID to_uuid = graph_->makeUUID(to_uuid_tmp);

    Output* from = graph_->findConnector<Output>(from_uuid);
    if(from == nullptr) {
        std::cerr << "cannot load fulcrum, connector with uuid '" << from_uuid << "' doesn't exist." << std::endl;
        return;
    }

    Input* to = graph_->findConnector<Input>(to_uuid);
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

void GraphIO::loadMessageConnection(Connectable* from, NodeHandle* parent, const UUID& to_uuid)
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
            ConnectionPtr c = BundledConnection::connect(
                        out, in,
                        parent->getOutputTransition(), target->getInputTransition());
            graph_->addConnection(c);
        }

    } catch(const std::exception& e) {
        std::cerr << "cannot load connection: " << e.what() << std::endl;
    }
}


void GraphIO::loadSignalConnection(Connectable* from, const UUID& to_uuid)
{
    try {
        NodeHandle* target = graph_->findNodeHandleForConnector(to_uuid);

        Slot* in = target->getSlot(to_uuid);
        if(!in) {
            std::cerr << "cannot load message connection from " << from->getUUID() << " to " << to_uuid << ", slot doesn't exist." << std::endl;
            return;
        }

        Trigger* out = dynamic_cast<Trigger*>(from);
        if(out && in) {
            ConnectionPtr c = SignalConnection::connect(
                        out, in);
            graph_->addConnection(c);
        }

    } catch(const std::exception& e) {
        std::cerr << "cannot load connection: " << e.what() << std::endl;
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

    NodeState::Ptr s = node_handle->getNodeState();
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
