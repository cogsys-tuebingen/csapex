/// HEADER
#include <csapex/model/node_modifier.h>

/// COMPONENT
#include <csapex/model/node.h>

using namespace csapex;

NodeModifier::NodeModifier(Node *node)
    : node_(node)
{

}

ConnectorIn* NodeModifier::addInput(ConnectionTypePtr type, const std::string& label, bool optional, bool async)
{
    return node_->addInput(type, label, optional, async);
}

ConnectorOut* NodeModifier::addOutput(ConnectionTypePtr type, const std::string& label)
{
    return node_->addOutput(type, label);
}
