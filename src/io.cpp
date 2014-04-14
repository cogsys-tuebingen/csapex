/// HEADER
#include <utils_param/io.h>

/// COMPONENT
#include <utils_param/parameter_factory.h>

using namespace param;
using namespace YAML;

#if NEW_YAML_API
Node YAML::convert<param::Parameter::Ptr>::encode(const param::Parameter::Ptr& rhs) {
    throw std::runtime_error("YAML::encode not implemented");
}

bool YAML::convert<param::Parameter::Ptr>::decode(const Node& node, param::Parameter::Ptr& rhs) {
    std::string type;

    if(YAML::exists(node, "type")) {
        node["type"] >> type;
    } else {
        type = "range";
    }

    rhs = ParameterFactory::makeEmpty(type);
    rhs->read(node);
    return true;
}

template<>
struct convert<param::Parameter::Ptr>;
#endif

Emitter& YAML::operator << (Emitter& e, const Parameter& p) {
    p.write(e);
    return e;
}
Emitter& YAML::operator << (Emitter& e, const Parameter::Ptr& p) {
    p->write(e);
    return e;
}

void YAML::operator >> (const Node& node, Parameter::Ptr& value) {
    std::string type;

    if(YAML::exists(node, "type")) {
        node["type"] >> type;
    } else {
        type = "range";
    }

    value = ParameterFactory::makeEmpty(type);
    value->read(node);
}
