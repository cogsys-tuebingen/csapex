/// HEADER
#include <csapex/model/connection_type.h>

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
ConnectionType::Flags::Flags()
    : data(0)
{

}

ConnectionType::ConnectionType(const std::string& type_name)
    : type_name_(type_name), seq_no_(-1)
{
    setDescriptiveName(type_name);
}

ConnectionType::~ConnectionType()
{
}

void ConnectionType::setDescriptiveName(const std::string &name)
{
    descriptive_name_ = name;
}

bool ConnectionType::canConnectTo(const ConnectionType *other_side) const
{
    return other_side->acceptsConnectionFrom(this);
}

bool ConnectionType::acceptsConnectionFrom(const ConnectionType *other_side) const
{
    return type_name_ == other_side->typeName();
}

std::string ConnectionType::descriptiveName() const
{
    return descriptive_name_;
}

std::string ConnectionType::typeName() const
{
    return type_name_;
}

int ConnectionType::sequenceNumber() const
{
    return seq_no_;
}

void ConnectionType::setSequenceNumber(int seq_no) const
{
    seq_no_ = seq_no;
}

bool ConnectionType::isValid() const
{
    return true;
}

bool ConnectionType::isContainer() const
{
    return false;
}

ConnectionType::Ptr ConnectionType::nestedType() const
{
    throw std::logic_error("cannot get nested type for non-container messages");
}
ConnectionType::ConstPtr ConnectionType::nestedValue(std::size_t index) const
{
    throw std::logic_error("cannot get nested value for non-container messages");
}
std::size_t ConnectionType::nestedValueCount() const
{
    throw std::logic_error("cannot get nested count for non-container messages");
}
void ConnectionType::addNestedValue(const ConstPtr &msg)
{
    throw std::logic_error("cannot add nested value to non-container messages");
}

void ConnectionType::writeRaw(const std::string &/*file*/, const std::string &/*base*/, const std::string& /*suffix*/) const
{
    std::cerr << "error: writeRaw not implemented for message type " << descriptiveName() << std::endl;
}
