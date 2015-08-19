/// HEADER
#include <csapex/view/node/node_adapter_builder.h>

using namespace csapex;

NodeAdapterBuilder::~NodeAdapterBuilder()
{

}

void NodeAdapterBuilder::setType(const std::string &type)
{
    type_ = type;
}

std::string NodeAdapterBuilder::getType() const
{
    return type_;
}

