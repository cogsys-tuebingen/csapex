/// HEADER
#include <csapex_core_plugins/int_message.h>

using namespace csapex;
using namespace connection_types;

IntMessage::IntMessage()
    : MessageTemplate<int, IntMessage> ("int")
{}

ConnectionType::Ptr IntMessage::make(){
    Ptr new_msg(new IntMessage);
    return new_msg;
}

void IntMessage::writeYaml(YAML::Emitter& yaml) {
    yaml << YAML::Key << "value" << YAML::Value << value;
}
void IntMessage::readYaml(const YAML::Node& node) {
    node["value"] >> value;
}
