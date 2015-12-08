#ifndef IO_H
#define IO_H

/// COMPONENT
#include <csapex/param/parameter.h>

/// SYSTEM
#include <yaml-cpp/yaml.h>

namespace YAML {

template<>
struct convert<csapex::param::Parameter::Ptr> {
    static Node encode(const csapex::param::Parameter::Ptr& rhs);

    static bool decode(const Node& node, csapex::param::Parameter::Ptr& rhs);
};

}

#endif // IO_H
