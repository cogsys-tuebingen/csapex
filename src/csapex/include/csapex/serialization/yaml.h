#ifndef YAML_HPP
#define YAML_HPP

/// PROJECT
#include <csapex/model/model_fwd.h>

/// SYSTEM
#include <yaml-cpp/yaml.h>

/// YAML
namespace YAML {
template<>
struct convert<csapex::Token> {
    static Node encode(const csapex::Token& rhs);
    static bool decode(const Node& node, csapex::Token& rhs);
};
template<>
struct convert<csapex::TokenConstPtr> {
    static Node encode(const csapex::TokenConstPtr& rhs);
    static bool decode(const Node& node, csapex::TokenConstPtr& rhs);
};
}

#endif // YAML_HPP

