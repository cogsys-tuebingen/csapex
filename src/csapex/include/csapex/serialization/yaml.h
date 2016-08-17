#ifndef YAML_HPP
#define YAML_HPP

/// PROJECT
#include <csapex/model/model_fwd.h>
#include <csapex/csapex_export.h>

/// SYSTEM
#include <yaml-cpp/yaml.h>

/// YAML
namespace YAML {
template<>
struct CSAPEX_EXPORT convert<csapex::TokenData> {
    static Node encode(const csapex::TokenData& rhs);
    static bool decode(const Node& node, csapex::TokenData& rhs);
};
template<>
struct CSAPEX_EXPORT convert<csapex::TokenDataConstPtr> {
    static Node encode(const csapex::TokenDataConstPtr& rhs);
    static bool decode(const Node& node, csapex::TokenDataConstPtr& rhs);
};
}

#endif // YAML_HPP

