#ifndef NODE_FACTORY_REMOTE_H
#define NODE_FACTORY_REMOTE_H

/// PROJECT
#include <csapex/factory/node_factory.h>
#include <csapex/factory/node_factory_local.h>
#include <csapex/io/remote.h>

namespace csapex
{

class CSAPEX_EXPORT NodeFactoryRemote: public NodeFactory, public Remote
{
public:
    NodeFactoryRemote(const SessionPtr &session);

protected:
    void ensureLoaded() override;
};

}

#endif // NODE_FACTORY_REMOTE_H
