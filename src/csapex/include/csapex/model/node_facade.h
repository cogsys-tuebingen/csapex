#ifndef NODE_FACADE_H
#define NODE_FACADE_H

/// COMPONENT
#include <csapex/model/model_fwd.h>

namespace csapex
{

class NodeFacade
{
public:
    NodeFacade(NodeHandleWeakPtr nh);
    NodeFacade(NodeHandleWeakPtr nh, NodeWorkerWeakPtr nw);

    NodeWorkerPtr getNodeWorker();
    NodeHandlePtr getNodeHandle();

private:
    NodeHandleWeakPtr nh_;
    NodeWorkerWeakPtr nw_;
};

}

#endif // NODE_FACADE_H
