/// HEADER
#include <csapex/param/io.h>

/// COMPONENT
#include <csapex/param/parameter_factory.h>

using namespace param;
using namespace YAML;

Node YAML::convert<param::Parameter::Ptr>::encode(const param::Parameter::Ptr& rhs) {
    YAML::Node n;
    rhs->serialize(n);
    return n;
}

bool YAML::convert<param::Parameter::Ptr>::decode(const Node& node, param::Parameter::Ptr& rhs) {
    std::string type;

    if(node["type"].IsDefined()) {
        type = node["type"].as<std::string>();
    } else {
        type = "range";
    }

    rhs = ParameterFactory::makeEmpty(type);
    rhs->deserialize(node);
    return true;
}

