#ifndef GRAPHIO_H
#define GRAPHIO_H

/// COMPONENT
#include <csapex/model/graph.h>

/// PROJECT
#include <csapex/factory/factory_fwd.h>

/// SYSTEM
#include <yaml-cpp/yaml.h>

namespace csapex
{

class GraphIO
{
public:
    GraphIO(Graph *graph, NodeFactory* node_factory);

public:
    void saveSettings(YAML::Node& yaml);
    void loadSettings(const YAML::Node& doc);

    void saveNodes(YAML::Node &yaml);
    void loadNode(const YAML::Node &doc);

    void saveConnections(YAML::Node &yaml);
    void loadConnections(const YAML::Node& doc);

protected:
    void serializeNode(YAML::Node& doc, NodeWorker *node_worker);
    void deserializeNode(const YAML::Node& doc, NodeWorker* node_worker);

private:
    Graph* graph_;
    NodeFactory* node_factory_;
};

}

#endif // GRAPHIO_H
