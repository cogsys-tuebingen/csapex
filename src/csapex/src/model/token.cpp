/// HEADER
#include <csapex/model/token.h>

/// COMPONENT
#include <csapex/msg/message.h>
#include <csapex/msg/message_traits.h>
#include <csapex/utility/assert.h>

/// SYSTEM
#include <iostream>

using namespace csapex;

/***
 * MESSAGE FLAGS
 */
Token::Flags::Flags()
    : data(0)
{

}

Token::Token(const std::string& type_name)
    : type_name_(type_name), seq_no_(-1)
{
    setDescriptiveName(type_name);
}

Token::~Token()
{
}

void Token::setDescriptiveName(const std::string &name)
{
    descriptive_name_ = name;
}

bool Token::canConnectTo(const Token *other_side) const
{
    return other_side->acceptsConnectionFrom(this);
}

bool Token::acceptsConnectionFrom(const Token *other_side) const
{
    return type_name_ == other_side->typeName();
}

std::string Token::descriptiveName() const
{
    return descriptive_name_;
}

std::string Token::typeName() const
{
    return type_name_;
}

int Token::sequenceNumber() const
{
    return seq_no_;
}

void Token::setSequenceNumber(int seq_no) const
{
    seq_no_ = seq_no;
}

bool Token::isValid() const
{
    return true;
}

bool Token::isContainer() const
{
    return false;
}

Token::Ptr Token::nestedType() const
{
    throw std::logic_error("cannot get nested type for non-container messages");
}
Token::ConstPtr Token::nestedValue(std::size_t index) const
{
    throw std::logic_error("cannot get nested value for non-container messages");
}
std::size_t Token::nestedValueCount() const
{
    throw std::logic_error("cannot get nested count for non-container messages");
}
void Token::addNestedValue(const ConstPtr &msg)
{
    throw std::logic_error("cannot add nested value to non-container messages");
}

void Token::writeRaw(const std::string &/*file*/, const std::string &/*base*/, const std::string& /*suffix*/) const
{
    std::cerr << "error: writeRaw not implemented for message type " << descriptiveName() << std::endl;
}
