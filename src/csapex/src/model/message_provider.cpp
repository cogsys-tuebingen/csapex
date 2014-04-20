/// HEADER
#include <csapex/model/message_provider.h>

using namespace csapex;

ConnectionType::ConstPtr MessageProvider::getType() const
{
    return type_;
}

void MessageProvider::setType(ConnectionType::Ptr type)
{
    type_ = type;
}

std::string MessageProvider::getName() const
{
    return name_;
}

void MessageProvider::setName(const std::string& name)
{
    name_ = name;
}

std::vector<param::Parameter::Ptr> MessageProvider::getParameters() const
{
    return state.getParameters();
}
