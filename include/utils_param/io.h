#ifndef IO_H
#define IO_H

/// COMPONENT
#include <utils_param/parameter.h>

/// SYSTEM
#include <yaml-cpp/yaml.h>

namespace YAML {

Emitter& operator << (Emitter& e, const param::Parameter& p);
Emitter& operator << (Emitter& e, const param::Parameter::Ptr& p);

void operator >> (const Node& node, param::Parameter::Ptr& value);

}

#endif // IO_H
