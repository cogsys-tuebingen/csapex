#ifndef DIRECT_NODE_WORKER_H
#define DIRECT_NODE_WORKER_H

/// COMPONENT
#include <csapex/model/node_worker.h>

namespace csapex
{
class CSAPEX_CORE_EXPORT DirectNodeWorker : public NodeWorker
{
public:
    DirectNodeWorker(NodeHandlePtr node_handle);
    ~DirectNodeWorker();
};

}  // namespace csapex

#endif  // DIRECT_NODE_WORKER_H
