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
    DesignerIO(Designer& designer);

    void saveSettings(YAML::Emitter &yaml);
    void loadSettings(YAML::Node& doc);

    void saveBoxes(YAML::Emitter &yaml, GraphPtr graph, WidgetController* widget_ctrl);
    void loadBoxes(YAML::Node& doc, WidgetController* widget_ctrl);

private:
    void saveBox(Node* node, WidgetController* widget_ctrl, YAML::Emitter &yaml);

private:
    Designer& designer_;
};

}

#endif // DESIGNERIO_H
