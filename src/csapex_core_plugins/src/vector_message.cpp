/// HEADER
#include <csapex_core_plugins/vector_message.h>

using namespace csapex;
using namespace connection_types;

VectorMessage::VectorMessage()
    : Message ("std::vector<?>")
{
    type_ = AnyMessage::make();
}
VectorMessage::VectorMessage(ConnectionType::Ptr type)
    : Message (std::string("std::vector<") + type->rawName()  + "::Ptr>")
{
    type_ = type;
}

ConnectionType::Ptr VectorMessage::getSubType() const
{
    return type_;
}

ConnectionType::Ptr VectorMessage::make(){
    Ptr new_msg(new VectorMessage);
    return new_msg;
}

ConnectionType::Ptr VectorMessage::clone() {
    Ptr new_msg(new VectorMessage);
    new_msg->value = value;
    return new_msg;
}

ConnectionType::Ptr VectorMessage::toType() {
    return make();
}

bool VectorMessage::canConnectTo(const ConnectionType *other_side) const
{
    const VectorMessage* vec = dynamic_cast<const VectorMessage*> (other_side);
    return vec != 0 && type_->canConnectTo(vec->getSubType().get());
}

bool VectorMessage::acceptsConnectionFrom(const ConnectionType *other_side) const
{
    const VectorMessage* vec = dynamic_cast<const VectorMessage*> (other_side);
    return vec != 0 && type_->acceptsConnectionFrom(vec->getSubType().get());
}

void VectorMessage::writeYaml(YAML::Emitter& yaml) {
    yaml << YAML::Key << "value" << YAML::Value << YAML::Flow;
    yaml << YAML::BeginSeq;
    for(unsigned i = 0; i < value.size(); ++i) {
        yaml << YAML::BeginMap;
        value[i]->writeYaml(yaml);
        yaml << YAML::EndMap;
    }
    yaml << YAML::EndSeq;
}
void VectorMessage::readYaml(const YAML::Node& node) {
    throw std::runtime_error("deserialization of vectors not implemented");
}
