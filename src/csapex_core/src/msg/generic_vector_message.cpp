/// HEADER
#include <csapex/msg/generic_vector_message.hpp>

/// COMPONENT
#include <csapex/msg/token_traits.h>
#include <csapex/utility/register_msg.h>
#include <csapex/serialization/yaml.h>
#include <csapex/msg/no_message.h>

CSAPEX_REGISTER_MESSAGE_WITH_NAME(csapex::connection_types::GenericVectorMessage, g_instance_generic_vector_)

using namespace csapex;
using namespace connection_types;

GenericVectorMessage::GenericVectorMessage(EntryInterface::Ptr impl, const std::string& frame_id, Message::Stamp stamp)
    : Message (type<GenericVectorMessage>::name(), frame_id, stamp), impl(impl)
{
}

GenericVectorMessage::GenericVectorMessage()
    : Message(type<GenericVectorMessage>::name(), "/", 0),
      impl(std::make_shared<InstancedImplementation>(std::make_shared<AnyMessage>()))
{

}

bool GenericVectorMessage::canConnectTo(const TokenData *other_side) const
{
    return impl->canConnectTo(other_side);
}

bool GenericVectorMessage::acceptsConnectionFrom(const TokenData *other_side) const
{
    return impl->acceptsConnectionFrom(other_side);
}

std::string GenericVectorMessage::descriptiveName() const
{
    return impl->descriptiveName();
}


/// ANYTHING

GenericVectorMessage::AnythingImplementation::AnythingImplementation()
    : EntryInterface("Anything")
{

}

void GenericVectorMessage::AnythingImplementation::encode(YAML::Node& node) const
{
    std::cout << "encode any" << std::endl;

}
void GenericVectorMessage::AnythingImplementation::decode(const YAML::Node& node)
{
    std::cout << "decode any" << std::endl;
}



bool GenericVectorMessage::AnythingImplementation::canConnectTo(const TokenData* other_side) const
{
    if(dynamic_cast<const EntryInterface*> (other_side)) {
        return true;
    } else if(dynamic_cast<const GenericVectorMessage*> (other_side)) {
        return true;
    } else {
        auto type = toType();
        return other_side->canConnectTo(type.get());
    }
}

bool GenericVectorMessage::AnythingImplementation::acceptsConnectionFrom(const TokenData *other_side) const
{
    if(dynamic_cast<const EntryInterface*> (other_side)) {
        return true;
    } else if(dynamic_cast<const GenericVectorMessage*> (other_side)) {
        return true;
    } else {
        return dynamic_cast<const AnyMessage*> (other_side) != nullptr;
    }
}

void GenericVectorMessage::AnythingImplementation::serialize(SerializationBuffer &data, SemanticVersion& version) const
{
    Message::serialize(data, version);
}
void GenericVectorMessage::AnythingImplementation::deserialize(const SerializationBuffer& data, const SemanticVersion& version)
{
    Message::deserialize(data, version);
}


// INSTANCED

GenericVectorMessage::InstancedImplementation::InstancedImplementation(TokenData::ConstPtr type)
    : EntryInterface("Anything"),
      type_(type)
{

}


GenericVectorMessage::InstancedImplementation::InstancedImplementation()
    : EntryInterface("Anything")
{
}


bool GenericVectorMessage::InstancedImplementation::canConnectTo(const TokenData* other_side) const
{
    if(const EntryInterface* ei = dynamic_cast<const EntryInterface*> (other_side)) {
        return nestedType()->canConnectTo(ei->nestedType().get());
    } else {
        const GenericVectorMessage* vec = dynamic_cast<const GenericVectorMessage*> (other_side);
        if(vec != 0) {
            return vec->canConnectTo(this);
        } else {
            auto type = nestedType();
            return other_side->canConnectTo(type.get());
            //return dynamic_cast<const AnyMessage*> (other_side) != nullptr;
        }
    }
}

bool GenericVectorMessage::InstancedImplementation::acceptsConnectionFrom(const TokenData *other_side) const
{
    if(const EntryInterface* ei = dynamic_cast<const EntryInterface*> (other_side)) {
        return nestedType()->canConnectTo(ei->nestedType().get());
    } else {
        return false;
    }
}

void GenericVectorMessage::InstancedImplementation::encode(YAML::Node& node) const
{
    std::cout << "vector instance: encode" << std::endl;
    node["value_type"] = type_->typeName();
    node["values"] = value;
}

void GenericVectorMessage::InstancedImplementation::decode(const YAML::Node& node)
{
    YAML::Emitter emitter;
    emitter << node;
    std::cout << "vector instance: decode " << emitter.c_str() << std::endl;
    value = node["values"].as< std::vector<TokenData::Ptr> >();
}

TokenData::Ptr GenericVectorMessage::InstancedImplementation::nestedType() const
{
    return type_->cloneAs<TokenData>();
}

void GenericVectorMessage::InstancedImplementation::addNestedValue(const TokenData::ConstPtr &msg)
{
    value.push_back(msg->cloneAs<TokenData>());
}
TokenData::ConstPtr GenericVectorMessage::InstancedImplementation::nestedValue(std::size_t i) const
{
    return value.at(i);
}
std::size_t GenericVectorMessage::InstancedImplementation::nestedValueCount() const
{
    return value.size();
}

void GenericVectorMessage::InstancedImplementation::serialize(SerializationBuffer &data, SemanticVersion& version) const
{
    data << value;
}
void GenericVectorMessage::InstancedImplementation::deserialize(const SerializationBuffer& data, const SemanticVersion& version)
{
    data >> value;
}

/// YAML
namespace YAML {
Node convert<csapex::connection_types::GenericVectorMessage>::encode(const csapex::connection_types::GenericVectorMessage& rhs)
{
    Node node = convert<csapex::connection_types::Message>::encode(rhs);
    rhs.encode(node);
    return node;
}

bool convert<csapex::connection_types::GenericVectorMessage>::decode(const Node& node, csapex::connection_types::GenericVectorMessage& rhs)
{
    if(!node.IsMap()) {
        return false;
    }
    convert<csapex::connection_types::Message>::decode(node, rhs);
    rhs.decode(node);
    return true;
}
}

