#ifndef DESIGNERIO_H
#define DESIGNERIO_H

/// COMPONENT
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

class DesignerIO
{
public:
    DesignerIO();

    void saveSettings(YAML::Node& yaml);
    void loadSettings(YAML::Node& doc);

    void saveBoxes(YAML::Node &yaml, Graph* graph, WidgetController* widget_ctrl);
    void loadBoxes(YAML::Node& doc, WidgetController* widget_ctrl);

private:
    void saveBox(NodeHandle *node, WidgetController* widget_ctrl, YAML::Node &yaml);
};

}

#endif // DESIGNERIO_H
