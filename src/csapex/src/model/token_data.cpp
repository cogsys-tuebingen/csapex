/// HEADER
#include <csapex/model/token.h>

/// COMPONENT
#include <csapex/msg/message.h>
#include <csapex/msg/token_traits.h>
#include <csapex/utility/assert.h>

/// SYSTEM
#include <iostream>

using namespace csapex;

TokenData::TokenData(const std::string& type_name)
    : type_name_(type_name)
{
    setDescriptiveName(type_name);
}

TokenData::~TokenData()
{
}

void TokenData::setDescriptiveName(const std::string &name)
{
    descriptive_name_ = name;
}

bool TokenData::canConnectTo(const TokenData *other_side) const
{
    return other_side->acceptsConnectionFrom(this);
}

bool TokenData::acceptsConnectionFrom(const TokenData *other_side) const
{
    return type_name_ == other_side->typeName();
}

std::string TokenData::descriptiveName() const
{
    return descriptive_name_;
}

std::string TokenData::typeName() const
{
    return type_name_;
}

TokenData::Ptr TokenData::clone() const
{
    return std::make_shared<TokenData>(*this);
}

TokenData::Ptr TokenData::toType() const
{
    return std::make_shared<TokenData>(type_name_);
}

bool TokenData::isValid() const
{
    return true;
}

bool TokenData::isContainer() const
{
    return false;
}

TokenData::Ptr TokenData::nestedType() const
{
    throw std::logic_error("cannot get nested type for non-container messages");
}
TokenData::ConstPtr TokenData::nestedValue(std::size_t index) const
{
    throw std::logic_error("cannot get nested value for non-container messages");
}
std::size_t TokenData::nestedValueCount() const
{
    throw std::logic_error("cannot get nested count for non-container messages");
}
void TokenData::addNestedValue(const ConstPtr &msg)
{
    throw std::logic_error("cannot add nested value to non-container messages");
}

void TokenData::writeRaw(const std::string &/*file*/, const std::string &/*base*/, const std::string& /*suffix*/) const
{
    std::cerr << "error: writeRaw not implemented for message type " << descriptiveName() << std::endl;
}
