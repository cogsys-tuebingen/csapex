/// HEADER
#include <csapex_core_plugins/double_message.h>

using namespace csapex;
using namespace connection_types;

DoubleMessage::DoubleMessage()
    : MessageTemplate<double, DoubleMessage> ("double")
{}

ConnectionType::Ptr DoubleMessage::make(){
    Ptr new_msg(new DoubleMessage);
    return new_msg;
}

void DoubleMessage::writeYaml(YAML::Emitter& yaml) {
    yaml << YAML::Key << "value" << YAML::Value << value;
}
void DoubleMessage::readYaml(const YAML::Node& node) {
    node["value"] >> value;
}
