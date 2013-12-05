/// HEADER
#include <csapex/model/generic_state.h>

/// PROJECT
#include <utils_param/io.h>

using namespace csapex;

void GenericState::writeYaml(YAML::Emitter& out) const {
    out << YAML::Key << "params" << YAML::Value << params;
}

void GenericState::readYaml(const YAML::Node& node) {
    if(node.FindValue("params")) {
        node["params"] >> params;
    }
}
