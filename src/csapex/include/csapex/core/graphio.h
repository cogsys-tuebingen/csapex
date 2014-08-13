#ifndef GRAPHIO_H
#define GRAPHIO_H

/// COMPONENT
#include <csapex/model/graph.h>
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <map>
#include <string>
#include <yaml-cpp/yaml.h>
#include <QList>
#include <QPoint>
#include <QSize>

/// FORWARD DECLARATION

namespace csapex
{

class GraphIO : public QObject
{
    Q_OBJECT

public:
    GraphIO(Graph* graph, BoxManager* node_factory);

public:
    void saveSettings(YAML::Emitter &yaml);
    void loadSettings(YAML::Node& doc);

    void saveNodes(YAML::Emitter &yaml);
    void loadNodes(YAML::Parser &doc);

    void saveConnections(YAML::Emitter& yaml);
    void loadConnections(YAML::Node& doc);

private:
    Graph* graph_;
    BoxManager* node_factory_;
};

}

#endif // GRAPHIO_H
