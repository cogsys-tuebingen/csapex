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

private:
    Designer& designer_;
};

}

#endif // DESIGNERIO_H
