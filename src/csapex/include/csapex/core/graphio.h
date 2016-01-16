#ifndef GRAPHIO_H
#define GRAPHIO_H

/// COMPONENT
#include <csapex/model/graph.h>

/// PROJECT
#include <csapex/factory/factory_fwd.h>
#include <csapex/data/point.h>

/// SYSTEM
#include <yaml-cpp/yaml.h>
#include <unordered_map>

namespace csapex
{

class GraphIO
{
public:
    GraphIO(Graph *graph, NodeFactory* node_factory);

public:
    void saveSettings(YAML::Node& yaml);
    void loadSettings(const YAML::Node& doc);

    void saveGraph(YAML::Node &yaml);
    void loadGraph(const YAML::Node &doc);

    void saveSelectedGraph(YAML::Node &yaml, const std::vector<UUID>& nodes);
    std::vector<UUID> loadIntoGraph(const YAML::Node &blueprint, const csapex::Point &position);


private:
    void saveNodes(YAML::Node &yaml);
    void loadNodes(const YAML::Node& doc);
    void loadNode(const YAML::Node &doc);


    void saveConnections(YAML::Node &yaml);
    void loadConnections(const YAML::Node& doc);
    void loadConnection(const YAML::Node& connection);

    void saveFulcrums(YAML::Node& fulcrum, const Connection* connection);
    void loadFulcrum(const YAML::Node& fulcrum);

protected:
    void saveNodes(YAML::Node &yaml, const std::vector<NodeHandle *> &nodes);
    void saveConnections(YAML::Node &yaml, const std::vector<ConnectionPtr> &connections);

    void serializeNode(YAML::Node& doc, NodeHandle* node_handle);
    void deserializeNode(const YAML::Node& doc, NodeHandlePtr node_handle);

    void loadMessageConnection(Connectable *from, NodeHandle *parent, const UUID &to_uuid);
    void loadSignalConnection(Connectable *from, const UUID &to_uuid);

    UUID readNodeUUID(const YAML::Node& doc);
    UUID readConnectorUUID(const YAML::Node& doc);

private:
    Graph* graph_;
    NodeFactory* node_factory_;

    std::unordered_map<UUID, UUID, UUID::Hasher> old_node_uuid_to_new_;
    double position_offset_x_;
    double position_offset_y_;
};

}

#endif // GRAPHIO_H
