#ifndef DESIGNERIO_H
#define DESIGNERIO_H

/// SYSTEM
#include <map>
#include <string>
#include <yaml-cpp/yaml.h>

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

    static void loadBoxShells(Designer* designer, const std::string &file, std::map<std::string, Box*>& loaded_boxes);
    static void loadConnections(Designer* designer, const std::string &file, std::map<std::string, Box*>& loaded_boxes);
};

}

#endif // DESIGNERIO_H
