/// HEADER
#include <csapex/factory/node_factory_remote.h>

using namespace csapex;

NodeFactoryRemote::NodeFactoryRemote(NodeFactoryLocal &tmp_ref)
    : tmp_ref_(tmp_ref)
{

}

bool NodeFactoryRemote::isValidType(const std::string& type) const
{
    return tmp_ref_.isValidType(type);
}

NodeConstructorPtr NodeFactoryRemote::getConstructor(const std::string& type)
{
    return tmp_ref_.getConstructor(type);
}
std::vector<NodeConstructorPtr> NodeFactoryRemote::getConstructors()
{
    return tmp_ref_.getConstructors();
}

std::map<std::string, std::vector<NodeConstructor::Ptr> > NodeFactoryRemote::getTagMap()
{
    return tmp_ref_.getTagMap();
}
