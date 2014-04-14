#ifndef IO_H
#define IO_H

/// COMPONENT
#include <utils_param/parameter.h>

/// SYSTEM
#include <utils_yaml/yamlplus.h>

namespace YAML {

#if NEW_YAML_API
template<>
struct convert<param::Parameter::Ptr> {
    static Node encode(const param::Parameter::Ptr& rhs);

    static bool decode(const Node& node, param::Parameter::Ptr& rhs);
};
#endif

Emitter& operator << (Emitter& e, const param::Parameter& p);
Emitter& operator << (Emitter& e, const param::Parameter::Ptr& p);

void operator >> (const Node& node, param::Parameter::Ptr& value);

}

#endif // IO_H
