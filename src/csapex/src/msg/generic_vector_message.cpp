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
        return dynamic_cast<const AnyMessage*> (other_side) != nullptr;
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

