/// HEADER
#include <csapex/model/node_facade.h>

/// PROJECT
#include <csapex/model/node_handle.h>
#include <csapex/model/node_worker.h>
#include <csapex/model/node_state.h>

/// SYSTEM
#include <iostream>

using namespace csapex;

NodeFacade::NodeFacade(NodeHandlePtr nh, NodeWorkerPtr nw)
    : nh_(nh), nw_(nw)
{
    observe(nw->start_profiling, [this](NodeWorker*) {
        start_profiling(this);
    });
    observe(nw->stop_profiling,  [this](NodeWorker*) {
        stop_profiling(this);
    });

    observe(nw->destroyed, destroyed);
    observe(nw->notification, notification);

    observe(nw->interval_start, [this](NodeWorker*, ActivityType type, std::shared_ptr<const Interval> stamp) {
        interval_start(this, type, stamp);
    });
    observe(nw->interval_end,  [this](NodeWorker*, std::shared_ptr<const Interval> stamp) {
        interval_end(this, stamp);
    });
}

NodeFacade::NodeFacade(NodeHandlePtr nh)
    : nh_(nh)
{

}

NodeFacade::~NodeFacade()
{
}

std::string NodeFacade::getType() const
{
    return nh_->getType();
}

UUID NodeFacade::getUUID() const
{
    return nh_->getUUID();
}

bool NodeFacade::isActive() const
{
    return nh_->isActive();
}

bool NodeFacade::isProcessingEnabled() const
{
    if(nw_) {
        return nw_->isProcessingEnabled();
    } else {
        return false;
    }
}
bool NodeFacade::isProfiling() const
{
    if(nw_) {
        return nw_->isProfiling();
    } else {
        return false;
    }
}

std::string NodeFacade::getLabel() const
{
    return nh_->getNodeState()->getLabel();
}


NodeWorkerPtr NodeFacade::getNodeWorker()
{
    return nw_;
}

NodeHandlePtr NodeFacade::getNodeHandle()
{
    return nh_;
}
