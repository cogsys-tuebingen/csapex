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

TokenData::Ptr GenericVectorMessage::clone() const
{
    Ptr new_msg(new GenericVectorMessage(impl->cloneEntry(), frame_id, impl->stamp_micro_seconds));
    return new_msg;
}

TokenData::Ptr GenericVectorMessage::toType() const
{
    Ptr new_msg(new GenericVectorMessage(impl->cloneEntry(), frame_id, 0));
    return new_msg;
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

GenericVectorMessage::EntryInterface::Ptr GenericVectorMessage::AnythingImplementation::cloneEntry() const
{
    return std::make_shared<GenericVectorMessage::AnythingImplementation>();
}

void GenericVectorMessage::AnythingImplementation::encode(YAML::Node& node) const
{

}
void GenericVectorMessage::AnythingImplementation::decode(const YAML::Node& node)
{

}



TokenData::Ptr GenericVectorMessage::AnythingImplementation::toType() const
{
    return std::make_shared<AnyMessage>();
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


// INSTANCED

GenericVectorMessage::InstancedImplementation::InstancedImplementation(TokenData::ConstPtr type)
    : EntryInterface("Anything"),
      type_(type)
{

}

GenericVectorMessage::EntryInterface::Ptr GenericVectorMessage::InstancedImplementation::cloneEntry() const
{
    auto res = std::make_shared<GenericVectorMessage::InstancedImplementation>(type_);
    res->value = value;
    return res;
}


TokenData::Ptr GenericVectorMessage::InstancedImplementation::toType() const
{
    return type_->clone();
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
    node["value_type"] = type_->typeName();
    node["values"] = value;
}

void GenericVectorMessage::InstancedImplementation::decode(const YAML::Node& node)
{
    value = node["values"].as< std::vector<TokenData::Ptr> >();
}

TokenData::Ptr GenericVectorMessage::InstancedImplementation::nestedType() const
{
    return type_->clone();
}

void GenericVectorMessage::InstancedImplementation::addNestedValue(const TokenData::ConstPtr &msg)
{
    value.push_back(msg->clone());
}
TokenData::ConstPtr GenericVectorMessage::InstancedImplementation::nestedValue(std::size_t i) const
{
    return value.at(i);
}
std::size_t GenericVectorMessage::InstancedImplementation::nestedValueCount() const
{
    return value.size();
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

