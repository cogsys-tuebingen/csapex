/// HEADER
#include <csapex/view/node/node_adapter_builder.h>

/// PROJECT
#include <csapex/model/node_facade_local.h>
#include <csapex/model/node_facade_remote.h>

using namespace csapex;

NodePtr csapex::impl::getNode(const NodeFacadePtr& facade)
{
    auto lf = castToLocal(facade);
    if(lf) {
        return lf->getNode();
    } else {
        return nullptr;
    }
}


NodeFacadeRemotePtr csapex::impl::castToRemote(const NodeFacadePtr& facade)
{
    return std::dynamic_pointer_cast<csapex::NodeFacadeRemote>(facade);
}
NodeFacadeLocalPtr csapex::impl::castToLocal(const NodeFacadePtr& facade)
{
    return std::dynamic_pointer_cast<csapex::NodeFacadeLocal>(facade);
}


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
