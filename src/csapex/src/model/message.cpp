/// HEADAER
#include <csapex/model/message.h>

/// SYSTEM
#include <cxxabi.h>

using namespace csapex;
using namespace connection_types;

std::string csapex::connection_types::type2name(const std::type_info& info)
{
    int status;
    std::string full_name(abi::__cxa_demangle(info.name(), 0, 0, &status));

    std::string replace = "csapex::";

    std::size_t ptr_pos = full_name.find(replace);
    if(ptr_pos != std::string::npos) {
        full_name.replace(ptr_pos, replace.size(), "");
    }

    return full_name;
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
