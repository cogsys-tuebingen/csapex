#ifndef YAML_HPP
#define YAML_HPP

/// PROJECT
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <yaml-cpp/yaml.h>

/// YAML
namespace YAML {
template<>
struct convert<csapex::ConnectionType> {
    static Node encode(const csapex::ConnectionType& rhs);
    static bool decode(const Node& node, csapex::ConnectionType& rhs);
};
}

#endif // YAML_HPP

