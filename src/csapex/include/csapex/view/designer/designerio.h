#ifndef DESIGNERIO_H
#define DESIGNERIO_H

/// COMPONENT
#include <csapex/view/csapex_qt_export.h>
#include <csapex/model/model_fwd.h>
#include <csapex/view/view_fwd.h>

/// SYSTEM
#include <map>
#include <string>

namespace YAML {
class Node;
}

namespace csapex
{

class CSAPEX_QT_EXPORT DesignerIO
{
public:
    DesignerIO();

    void saveSettings(YAML::Node& yaml);
    void loadSettings(YAML::Node& doc);

    void saveBoxes(YAML::Node &yaml, Graph* graph, GraphView *view);
    void loadBoxes(YAML::Node& doc, GraphView *view);

private:
    void saveBox(NodeHandle *node, GraphView *view, YAML::Node &yaml);
};

}

#endif // DESIGNERIO_H
