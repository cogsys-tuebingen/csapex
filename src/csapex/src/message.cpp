/// HEADAER
#include <csapex/message.h>

using namespace csapex;
using namespace connection_types;

Message::Message(const std::string& name)
    : ConnectionType(name)
{
}

void Message::writeYaml(YAML::Emitter& yaml) {

}
void Message::readYaml(YAML::Node& node) {

}


AnyMessage::AnyMessage(const std::string& name)
    : Message(name)
{}

ConnectionType::Ptr AnyMessage::clone() {
    AnyMessage::Ptr new_msg(new AnyMessage("anything"));
    return new_msg;
}

ConnectionType::Ptr AnyMessage::toType() {
    Ptr new_msg(new AnyMessage("anything"));
    return new_msg;
}

ConnectionType::Ptr AnyMessage::make(){
    Ptr new_msg(new AnyMessage("anything"));
    return new_msg;
}

bool AnyMessage::canConnectTo(Ptr other_side) {
    return true;
}

bool AnyMessage::acceptsConnectionFrom(ConnectionType* other_side) {
    return true;
}
