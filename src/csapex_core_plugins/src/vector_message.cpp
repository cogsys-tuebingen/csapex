/// HEADER
#include <csapex_core_plugins/vector_message.h>

using namespace csapex;
using namespace connection_types;

GenericVectorMessage::GenericVectorMessage()
    : Message ("std::vector<?>")
{
    type_ = &typeid(void);
}
GenericVectorMessage::GenericVectorMessage(const std::type_info &type)
    : Message (std::string("std::vector<") + type2name(type)  + ">")
{
    type_ = &type;
}

GenericVectorMessage::Ptr GenericVectorMessage::make(){
    Ptr new_msg(new GenericVectorMessage);
    return new_msg;
}

ConnectionType::Ptr GenericVectorMessage::clone() {
    Ptr new_msg(new GenericVectorMessage);
    new_msg->value = value;
    return new_msg;
}

ConnectionType::Ptr GenericVectorMessage::toType() {
    return make();
}

bool GenericVectorMessage::canConnectTo(const ConnectionType *other_side) const
{
    const GenericVectorMessage* vec = dynamic_cast<const GenericVectorMessage*> (other_side);
    return vec != 0 && type_ == vec->type_;
}

bool GenericVectorMessage::acceptsConnectionFrom(const ConnectionType *other_side) const
{
    const GenericVectorMessage* vec = dynamic_cast<const GenericVectorMessage*> (other_side);
    return vec != 0 && type_ == vec->type_;
}

void GenericVectorMessage::writeYaml(YAML::Emitter& yaml) {
    throw std::runtime_error("serialization of generic vectors not implemented");
}
void GenericVectorMessage::readYaml(const YAML::Node& node) {
    throw std::runtime_error("deserialization of generic vectors not implemented");
}


//// OLD



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

VectorMessage::Ptr VectorMessage::make(){
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
