/// HEADER
#include <csapex/model/node_modifier.h>

/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/msg/message_factory.h>

using namespace csapex;

NodeModifier::NodeModifier(Node *node)
    : node_(node)
{

}

Input* NodeModifier::addInput(ConnectionTypePtr type, const std::string& label, bool optional, bool async)
{
    return node_->addInput(type, label, optional, async);
}

Output* NodeModifier::addOutput(ConnectionTypePtr type, const std::string& label)
{
    return node_->addOutput(type, label);
}
