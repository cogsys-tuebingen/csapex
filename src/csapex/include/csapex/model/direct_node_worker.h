#ifndef DIRECT_NODE_WORKER_H
#define DIRECT_NODE_WORKER_H

/// COMPONENT
#include <csapex/model/node_worker.h>

namespace csapex
{
class CSAPEX_EXPORT DirectNodeWorker : public NodeWorker
{
public:
    DirectNodeWorker(NodeHandlePtr node_handle);
    ~DirectNodeWorker();
};

}

#endif // DIRECT_NODE_WORKER_H
