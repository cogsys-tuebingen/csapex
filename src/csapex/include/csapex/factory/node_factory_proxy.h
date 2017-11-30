#ifndef NODE_FACTORY_PROXY_H
#define NODE_FACTORY_PROXY_H

/// PROJECT
#include <csapex/factory/node_factory.h>
#include <csapex/factory/node_factory_impl.h>
#include <csapex/io/proxy.h>

namespace csapex
{

class CSAPEX_EXPORT NodeFactoryProxy: public NodeFactory, public Proxy
{
public:
    NodeFactoryProxy(const SessionPtr &session);

protected:
    void ensureLoaded() override;
};

}

#endif // NODE_FACTORY_PROXY_H
