#ifndef GRAPHIO_H
#define GRAPHIO_H

/// COMPONENT
#include <csapex/model/graph.h>

/// PROJECT
#include <csapex/factory/factory_fwd.h>
#include <csapex/data/point.h>
#include <csapex/profiling/profilable.h>
#include <csapex/serialization/serialization_fwd.h>

/// SYSTEM
#include <yaml-cpp/yaml.h>
#include <unordered_map>

namespace csapex
{

class CSAPEX_EXPORT GraphIO : public Profilable
{
public:
    GraphIO(SubgraphNode *graph, NodeFactory* node_factory);

public:
    // options
    void setIgnoreForwardingConnections(bool ignore);

    // api
    void saveSettings(YAML::Node& yaml);
    void loadSettings(const YAML::Node& doc);

    Snippet saveGraph();
    void saveGraphTo(YAML::Node &yaml);

    void loadGraph(const Snippet &doc);
    void loadGraphFrom(const YAML::Node &doc);

    Snippet saveSelectedGraph(const std::vector<UUID>& nodes);

    std::unordered_map<UUID, UUID, UUID::Hasher>
    loadIntoGraph(const Snippet &blueprint, const csapex::Point &position);

public:
    csapex::slim_signal::Signal<void (SubgraphNode*, YAML::Node& e)> saveViewRequest;
    csapex::slim_signal::Signal<void (SubgraphNode*, YAML::Node& n)> loadViewRequest;

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

    void loadConnection(ConnectablePtr from, const UUID &to_uuid, const std::string& connection_type);

    UUID readNodeUUID(std::weak_ptr<UUIDProvider> parent, const YAML::Node& doc);
    UUID readConnectorUUID(std::weak_ptr<UUIDProvider> parent, const YAML::Node& doc);

private:
    SubgraphNode* graph_;
    NodeFactory* node_factory_;

    std::unordered_map<UUID, UUID, UUID::Hasher> old_node_uuid_to_new_;
    double position_offset_x_;
    double position_offset_y_;

    bool ignore_forwarding_connections_;
};

}

#endif // GRAPHIO_H
