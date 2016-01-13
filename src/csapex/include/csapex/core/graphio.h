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

    void saveGraph(YAML::Node &yaml);
    void loadGraph(const YAML::Node &doc);


private:
    void saveNodes(YAML::Node &yaml);
    void loadNodes(const YAML::Node& doc);

    void loadNode(const YAML::Node &doc);

    void saveConnections(YAML::Node &yaml);
    void loadConnections(const YAML::Node& doc);

protected:
    void serializeNode(YAML::Node& doc, NodeHandle* node_handle);
    void deserializeNode(const YAML::Node& doc, NodeHandlePtr node_handle);

    void loadMessageConnection(Connectable *from, NodeHandle *parent, const UUID &to_uuid);
    void loadSignalConnection(Connectable *from, const UUID &to_uuid);

private:
    Graph* graph_;
    NodeFactory* node_factory_;
};

}

#endif // GRAPHIO_H
