/// HEADAER
#include <csapex/model/message.h>

/// SYSTEM
#include <cxxabi.h>

using namespace csapex;
using namespace connection_types;

namespace {
std::string replace(const std::string& s, const std::string& find, const std::string& replace) {
    std::string result = s;
    std::size_t ptr_pos = s.find(find);
    if(ptr_pos != std::string::npos) {
        result.replace(ptr_pos, find.size(), replace);
    }
    return result;
}
}

std::string csapex::connection_types::type2name(const std::type_info& info)
{
    int status;
    std::string full_name(abi::__cxa_demangle(info.name(), 0, 0, &status));

    return replace(replace(full_name, "connection_types::", ""), "csapex::", "");
}

Message::Message(const std::string& name)
    : ConnectionType(name)
{
}

Message::~Message()
{

}

void Message::writeYaml(YAML::Emitter&)
{

}
void Message::readYaml(const YAML::Node&)
{

}


AnyMessage::AnyMessage()
    : Message("anything")
{}

ConnectionType::Ptr AnyMessage::clone()
{
    AnyMessage::Ptr new_msg(new AnyMessage);
    return new_msg;
}

ConnectionType::Ptr AnyMessage::toType()
{
    Ptr new_msg(new AnyMessage);
    return new_msg;
}

ConnectionType::Ptr AnyMessage::make()
{
    Ptr new_msg(new AnyMessage);
    return new_msg;
}

bool AnyMessage::canConnectTo(const ConnectionType*) const
{
    return true;
}

bool AnyMessage::acceptsConnectionFrom(const ConnectionType*) const
{
    return true;
}
