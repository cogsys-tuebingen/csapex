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
    DesignerIO(Designer& designer, GraphPtr graph, WidgetController *widget_ctrl);

    void saveSettings(YAML::Emitter &yaml);
    void loadSettings(YAML::Node& doc);

    void saveBoxes(YAML::Emitter &yaml);
    void loadBoxes(YAML::Node& doc);

private:
    void saveBox(Node* node, YAML::Emitter &yaml);
    void loadBox(Node* node, YAML::Node &doc);

private:
    Designer& designer_;
    GraphPtr graph_;
    WidgetController *widget_ctrl_;
};

}

#endif // DESIGNERIO_H
