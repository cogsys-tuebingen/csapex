#ifndef IO_H
#define IO_H

/// COMPONENT
#include <csapex/param/parameter.h>

/// SYSTEM
#include <yaml-cpp/yaml.h>

namespace YAML {

template<>
struct convert<param::Parameter::Ptr> {
    static Node encode(const param::Parameter::Ptr& rhs);

    static bool decode(const Node& node, param::Parameter::Ptr& rhs);
};

}

#endif // IO_H
