/// HEADER
#include <csapex_core_plugins/string_message.h>

using namespace csapex;
using namespace connection_types;

StringMessage::StringMessage()
    : MessageTemplate<std::string, StringMessage> ("std::string")
{}

ConnectionType::Ptr StringMessage::make(){
    Ptr new_msg(new StringMessage);
    return new_msg;
}

void StringMessage::writeYaml(YAML::Emitter& yaml) {
    yaml << YAML::Key << "value" << YAML::Value << value;
}
void StringMessage::readYaml(const YAML::Node& node) {
    node["value"] >> value;
}
