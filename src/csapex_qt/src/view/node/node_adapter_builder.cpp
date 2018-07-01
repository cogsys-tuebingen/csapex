/// HEADER
#include <csapex/view/node/node_adapter_builder.h>

/// PROJECT
#include <csapex/model/node_facade_impl.h>
#include <csapex/model/node_facade_proxy.h>

using namespace csapex;

NodePtr csapex::impl::getNode(const NodeFacadePtr& facade)
{
    auto lf = castToImplementation(facade);
    if (lf) {
        return lf->getNode();
    } else {
        return nullptr;
    }
}

NodeFacadeProxyPtr csapex::impl::castToProxy(const NodeFacadePtr& facade)
{
    return std::dynamic_pointer_cast<csapex::NodeFacadeProxy>(facade);
}
NodeFacadeImplementationPtr csapex::impl::castToImplementation(const NodeFacadePtr& facade)
{
    return std::dynamic_pointer_cast<csapex::NodeFacadeImplementation>(facade);
}

NodeAdapterBuilder::~NodeAdapterBuilder()
{
}

void NodeAdapterBuilder::setType(const std::string& type)
{
    type_ = type;
}

std::string NodeAdapterBuilder::getType() const
{
    return type_;
}
