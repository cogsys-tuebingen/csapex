#ifndef DESIGNERIO_H
#define DESIGNERIO_H

/// SYSTEM
#include <map>
#include <string>
#include <yaml-cpp/yaml.h>
#include <QList>

/// FORWARD DECLARATION

namespace vision_evaluator
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

public:
    static void save(Designer* designer, const std::string& file);
    static void load(Designer* designer, const std::string &file);

private:
    static void loadSettings(Designer* designer, YAML::Node& doc);

    static void loadBoxes(Designer* designer, const std::string &file);

    static void saveConnections(YAML::Emitter& yaml, QList<Box *> &boxes);
    static void loadConnections(Designer* designer, const std::string &file);
};

}

#endif // DESIGNERIO_H
