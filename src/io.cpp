/// HEADER
#include <utils_param/io.h>

/// COMPONENT
#include <utils_param/parameter_factory.h>

using namespace param;
using namespace YAML;

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
    if(node.FindValue("type")) {
        node["type"] >> type;
    } else {
        type = "range";
    }

    value = ParameterFactory::makeEmpty(type);
    value->read(node);
}
