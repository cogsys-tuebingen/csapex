#ifndef DESIGNERIO_H
#define DESIGNERIO_H

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
class Box;
class Designer;

class DesignerIO
{
private:
    DesignerIO();

public:
    static const std::string extension;
    static const std::string default_config;
    static const std::string config_selector;

    static std::string defaultConfigFile();
    static std::string defaultConfigPath();

public:
    static void createDefaultConfig(const std::string &file);

    static void save(Designer* designer, const std::string& file);
    static void load(Designer* designer, const std::string &file);

private:
    static void saveSettings(QPoint window_pos, QSize window_size, YAML::Emitter &yaml);
    static void loadSettings(Designer* designer, YAML::Node& doc);

    static void loadBoxes(Designer* designer, const std::string &file);

    static void saveConnections(YAML::Emitter& yaml, QList<Box *> &boxes);
    static void loadConnections(Designer* designer, const std::string &file);
};

}

#endif // DESIGNERIO_H
