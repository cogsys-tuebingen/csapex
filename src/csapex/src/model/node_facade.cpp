/// HEADER
#include <csapex/model/node_facade.h>

/// PROJECT
#include <csapex/model/node_handle.h>
#include <csapex/model/node_worker.h>
#include <csapex/model/node_state.h>
#include <csapex/msg/input_transition.h>
#include <csapex/msg/output_transition.h>
#include <csapex/signal/event.h>
#include <csapex/model/graph/vertex.h>

/// SYSTEM
#include <iostream>
#include <sstream>

using namespace csapex;

NodeFacade::NodeFacade(NodeHandlePtr nh, NodeWorkerPtr nw)
    : NodeFacade(nh)
{
    nw_ = nw;

    observe(nw->start_profiling, [this](NodeWorker*) {
        start_profiling(this);
    });
    observe(nw->stop_profiling,  [this](NodeWorker*) {
        stop_profiling(this);
    });

    observe(nw->destroyed, destroyed);
    observe(nw->notification, notification);

    observe(nw->messages_processed, messages_processed);

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
    observe(nh->connector_created, connector_created);
    observe(nh->connector_removed, connector_removed);
    observe(nh->node_state_changed, node_state_changed);


    observe(nh->connection_in_prograss, connection_in_prograss);
    observe(nh->connection_done, connection_done);
    observe(nh->connection_start, connection_start);

    // TODO: eliminate
    observe(execution_requested, nh->execution_requested);
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

bool NodeFacade::isGraph() const
{
    return nh_->isGraph();
}

AUUID NodeFacade::getSubgraphAUUID() const
{
    return nh_->getSubgraphAUUID();
}

bool NodeFacade::isSource() const
{
    return nh_->isSource();
}
bool NodeFacade::isSink() const
{
    return nh_->isSink();
}

bool NodeFacade::isVariadic() const
{
    return nh_->isVariadic();
}
bool NodeFacade::hasVariadicInputs() const
{
    return nh_->hasVariadicInputs();
}
bool NodeFacade::hasVariadicOutputs() const
{
    return nh_->hasVariadicOutputs();
}
bool NodeFacade::hasVariadicEvents() const
{
    return nh_->hasVariadicEvents();
}
bool NodeFacade::hasVariadicSlots() const
{
    return nh_->hasVariadicSlots();
}


NodeCharacteristics NodeFacade::getNodeCharacteristics() const
{
    return nh_->getVertex()->getNodeCharacteristics();
}

bool NodeFacade::isProfiling() const
{
    if(nw_) {
        return nw_->isProfiling();
    } else {
        return false;
    }
}
void NodeFacade::setProfiling(bool profiling)
{
    if(nw_) {
        nw_->setProfiling(profiling);
    }
}
bool NodeFacade::isError() const
{
    if(nw_) {
        return nw_->isError();
    } else {
        return false;
    }
}
ErrorState::ErrorLevel NodeFacade::errorLevel() const
{
    if(nw_) {
        return nw_->errorLevel();
    } else {
        return ErrorState::ErrorLevel::NONE;
    }
}
std::string NodeFacade::errorMessage() const
{
    if(nw_) {
        return nw_->errorMessage();
    } else {
        return {};
    }
}

ExecutionState NodeFacade::getExecutionState() const
{
    if(nw_) {
        return nw_->getExecutionState();
    } else {
        return ExecutionState::UNKNOWN;
    }
}


std::string NodeFacade::getLabel() const
{
    return nh_->getNodeState()->getLabel();
}

double NodeFacade::getExecutionFrequency() const
{
    return nh_->getRate().getEffectiveFrequency();
}

double NodeFacade::getMaximumFrequency() const
{
    return nh_->getNodeState()->getMaximumFrequency();
}

NodePtr NodeFacade::getNode()
{
    return nh_->getNode().lock();
}

NodeHandlePtr NodeFacade::getNodeHandle()
{
    return nh_;
}

NodeStatePtr NodeFacade::getNodeState() const
{
    return nh_->getNodeState();
}
NodeStatePtr NodeFacade::getNodeStateCopy() const
{
    return nh_->getNodeStateCopy();
}

ProfilerPtr NodeFacade::getProfiler()
{
    if(nw_) {
        return nw_->getProfiler();
    } else {
        return {};
    }
}

std::string NodeFacade::getDebugDescription() const
{
    OutputTransition* ot = nh_->getOutputTransition();
    InputTransition* it = nh_->getInputTransition();

    std::stringstream ss;
    ss << ", it: ";
    ss << (it->isEnabled() ? "enabled" : "disabled");
    ss << ", ot: ";
    ss << (ot->isEnabled() ? "enabled" : "disabled");
    ss << ", events: ";
    bool events_enabled = true;
    for(EventPtr e : nh_->getExternalEvents()){
        if(!e->canReceiveToken()) {
            events_enabled = false;
            break;
        }
    }
    ss << (events_enabled ? "enabled" : "disabled");
    return ss.str();
}
