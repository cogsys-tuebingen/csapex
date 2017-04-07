/// HEADER
#include <csapex/model/node_facade.h>

/// PROJECT
#include <csapex/model/node_handle.h>
#include <csapex/model/node_worker.h>

using namespace csapex;

NodeFacade::NodeFacade(NodeHandleWeakPtr nh, NodeWorkerWeakPtr nw)
    : nh_(nh), nw_(nw)
{
    if(NodeWorkerPtr worker = nw.lock()) {
        observe(worker->start_profiling, [this](NodeWorker*) {
            start_profiling(this);
        });
        observe(worker->stop_profiling,  [this](NodeWorker*) {
            stop_profiling(this);
        });
    }
}

NodeFacade::NodeFacade(NodeHandleWeakPtr nh)
    : nh_(nh)
{

}

UUID NodeFacade::getUUID() const
{
    if(NodeHandlePtr nh = nh_.lock()) {
        return nh->getUUID();
    }

    throw std::runtime_error("state node handle");
}


NodeWorkerPtr NodeFacade::getNodeWorker()
{
    return nw_.lock();
}

NodeHandlePtr NodeFacade::getNodeHandle()
{
    return nh_.lock();
}
