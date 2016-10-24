/// HEADER
#include <csapex/model/generator_node.h>

using namespace csapex;

GeneratorNode::GeneratorNode()
{

}

void GeneratorNode::getProperties(std::vector<std::string>& properties) const
{
    Node::getProperties(properties);
    properties.push_back("source");
}

void GeneratorNode::notifyMessagesProcessed()
{

}


bool GeneratorNode::canProcess() const
{
    return true;
}
