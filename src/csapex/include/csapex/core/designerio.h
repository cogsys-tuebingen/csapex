#ifndef DESIGNERIO_H
#define DESIGNERIO_H

/// COMPONENT
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

class DesignerIO
{
public:
    DesignerIO();

    void saveSettings(YAML::Node& yaml);
    void loadSettings(YAML::Node& doc);

    void saveBoxes(YAML::Node &yaml, Graph* graph, WidgetController* widget_ctrl);
    void loadBoxes(YAML::Node& doc, WidgetController* widget_ctrl);

private:
    void saveBox(NodeWorker *node, WidgetController* widget_ctrl, YAML::Node &yaml);
};

}

#endif // DESIGNERIO_H
