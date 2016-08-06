#ifndef IO_H
#define IO_H

/// COMPONENT
#include <csapex/param/parameter.h>

/// SYSTEM
#include <yaml-cpp/yaml.h>

namespace YAML {

template<>
struct convert<csapex::param::ParameterPtr> {
    static Node encode(const csapex::param::ParameterPtr& rhs);

    static bool decode(const Node& node, csapex::param::ParameterPtr& rhs);
};

template<>
struct convert<csapex::param::ParameterConstPtr> {
    static Node encode(const csapex::param::ParameterConstPtr& rhs);

    static bool decode(const Node& node, csapex::param::ParameterConstPtr& rhs);
};

}

#endif // IO_H
