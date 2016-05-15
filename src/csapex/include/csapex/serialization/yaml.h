#ifndef YAML_HPP
#define YAML_HPP

/// PROJECT
#include <csapex/model/model_fwd.h>

/// SYSTEM
#include <yaml-cpp/yaml.h>

/// YAML
namespace YAML {
template<>
struct convert<csapex::TokenData> {
    static Node encode(const csapex::TokenData& rhs);
    static bool decode(const Node& node, csapex::TokenData& rhs);
};
template<>
struct convert<csapex::TokenDataConstPtr> {
    static Node encode(const csapex::TokenDataConstPtr& rhs);
    static bool decode(const Node& node, csapex::TokenDataConstPtr& rhs);
};
}

#endif // YAML_HPP

