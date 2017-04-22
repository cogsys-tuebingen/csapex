/// HEADER
#include <csapex/model/node_facade_remote.h>

/// PROJECT
#include <csapex/model/node_handle.h>
#include <csapex/model/node_worker.h>
#include <csapex/model/node_state.h>
#include <csapex/msg/input_transition.h>
#include <csapex/msg/output_transition.h>
#include <csapex/signal/event.h>
#include <csapex/model/graph/vertex.h>
#include <csapex/model/node.h>

/// SYSTEM
#include <iostream>
#include <sstream>

using namespace csapex;

#if 0
NodeFacadeRemote::NodeFacadeRemote(NodeHandlePtr nh, NodeWorkerPtr nw, NodeRunnerPtr nr)
{
    nw_ = nw;
    nr_ = nr;

    nh->setNodeRunner(nr_);

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

//NodeFacadeRemote::NodeFacadeRemote(NodeHandlePtr nh)
//{
//    observe(nh->connector_created, connector_created);
//    observe(nh->connector_removed, connector_removed);
//    observe(nh->node_state_changed, node_state_changed);


//    observe(nh->connection_in_prograss, connection_in_prograss);
//    observe(nh->connection_done, connection_done);
//    observe(nh->connection_start, connection_start);


//    observe(nh->parameters_changed, parameters_changed);
//}

NodeFacadeRemote::~NodeFacadeRemote()
{
}

std::string NodeFacadeRemote::getType() const
{
    return nh_->getType();
}

UUID NodeFacadeRemote::getUUID() const
{
    return nh_->getUUID();
}

bool NodeFacadeRemote::isActive() const
{
    return nh_->isActive();
}

void NodeFacadeRemote::setActive(bool active)
{
    nh_->setActive(active);
}

bool NodeFacadeRemote::isProcessingEnabled() const
{
    if(nw_) {
        return nw_->isProcessingEnabled();
    } else {
        return false;
    }
}

bool NodeFacadeRemote::isGraph() const
{
    return nh_->isGraph();
}

AUUID NodeFacadeRemote::getSubgraphAUUID() const
{
    return nh_->getSubgraphAUUID();
}

bool NodeFacadeRemote::isSource() const
{
    return nh_->isSource();
}
bool NodeFacadeRemote::isSink() const
{
    return nh_->isSink();
}

bool NodeFacadeRemote::isParameterInput(const UUID& id)
{
    return nh_->isParameterInput(id);
}

bool NodeFacadeRemote::isParameterOutput(const UUID& id)
{
    return nh_->isParameterOutput(id);
}

bool NodeFacadeRemote::isVariadic() const
{
    return nh_->isVariadic();
}
bool NodeFacadeRemote::hasVariadicInputs() const
{
    return nh_->hasVariadicInputs();
}
bool NodeFacadeRemote::hasVariadicOutputs() const
{
    return nh_->hasVariadicOutputs();
}
bool NodeFacadeRemote::hasVariadicEvents() const
{
    return nh_->hasVariadicEvents();
}
bool NodeFacadeRemote::hasVariadicSlots() const
{
    return nh_->hasVariadicSlots();
}


std::vector<ConnectorDescription> NodeFacadeRemote::getInputs() const
{
    return nh_->getExternalInputDescriptions();
}

std::vector<ConnectorDescription> NodeFacadeRemote::getOutputs() const
{
    return nh_->getExternalOutputDescriptions();
}

std::vector<ConnectorDescription> NodeFacadeRemote::getEvents() const
{
    return nh_->getExternalEventDescriptions();
}

std::vector<ConnectorDescription> NodeFacadeRemote::getSlots() const
{
    return nh_->getExternalSlotDescriptions();
}


std::vector<ConnectorDescription> NodeFacadeRemote::getInternalInputs() const
{
    return nh_->getInternalInputDescriptions();
}

std::vector<ConnectorDescription> NodeFacadeRemote::getInternalOutputs() const
{
    return nh_->getInternalOutputDescriptions();
}

std::vector<ConnectorDescription> NodeFacadeRemote::getInternalEvents() const
{
    return nh_->getInternalEventDescriptions();
}

std::vector<ConnectorDescription> NodeFacadeRemote::getInternalSlots() const
{
    return nh_->getInternalSlotDescriptions();
}


NodeCharacteristics NodeFacadeRemote::getNodeCharacteristics() const
{
    return nh_->getVertex()->getNodeCharacteristics();
}

bool NodeFacadeRemote::isProfiling() const
{
    if(nw_) {
        return nw_->isProfiling();
    } else {
        return false;
    }
}
void NodeFacadeRemote::setProfiling(bool profiling)
{
    if(nw_) {
        nw_->setProfiling(profiling);
    }
}
bool NodeFacadeRemote::isError() const
{
    if(nw_) {
        return nw_->isError();
    } else {
        return false;
    }
}
ErrorState::ErrorLevel NodeFacadeRemote::errorLevel() const
{
    if(nw_) {
        return nw_->errorLevel();
    } else {
        return ErrorState::ErrorLevel::NONE;
    }
}
std::string NodeFacadeRemote::errorMessage() const
{
    if(nw_) {
        return nw_->errorMessage();
    } else {
        return {};
    }
}

ExecutionState NodeFacadeRemote::getExecutionState() const
{
    if(nw_) {
        return nw_->getExecutionState();
    } else {
        return ExecutionState::UNKNOWN;
    }
}


std::string NodeFacadeRemote::getLabel() const
{
    return nh_->getNodeState()->getLabel();
}

double NodeFacadeRemote::getExecutionFrequency() const
{
    return nh_->getRate().getEffectiveFrequency();
}

double NodeFacadeRemote::getMaximumFrequency() const
{
    return nh_->getNodeState()->getMaximumFrequency();
}

NodeHandlePtr NodeFacadeRemote::getNodeHandle() const
{
    return nh_;
}

NodePtr NodeFacadeRemote::getNode() const
{
    return nh_->getNode().lock();
}

NodeWorkerPtr NodeFacadeRemote::getNodeWorker() const
{
    return nw_;
}

NodeRunnerPtr NodeFacadeRemote::getNodeRunner() const
{
    return nr_;
}

NodeStatePtr NodeFacadeRemote::getNodeState() const
{
    return nh_->getNodeState();
}
NodeStatePtr NodeFacadeRemote::getNodeStateCopy() const
{
    return nh_->getNodeStateCopy();
}

void NodeFacadeRemote::setNodeState(NodeStatePtr memento)
{
    nh_->setNodeState(memento);
}

GenericStateConstPtr NodeFacadeRemote::getParameterState() const
{
    return nh_->getNode().lock()->getParameterStateClone();
}

ProfilerPtr NodeFacadeRemote::getProfiler()
{
    if(nw_) {
        return nw_->getProfiler();
    } else {
        return {};
    }
}

std::string NodeFacadeRemote::getDebugDescription() const
{
    OutputTransition* ot = nh_->getOutputTransition();
    InputTransition* it = nh_->getInputTransition();

    std::stringstream ss;
    ss << ", source: ";
    ss << (nh_->isSource() ? "yes" : "no");
    ss << ", sink: ";
    ss << (nh_->isSink() ? "yes" : "no");
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

std::string NodeFacadeRemote::getLoggerOutput(ErrorState::ErrorLevel level) const
{
    if(NodePtr node = nh_->getNode().lock()){
        switch(level) {
        case ErrorState::ErrorLevel::ERROR:
            return node->aerr.history().str();
        case ErrorState::ErrorLevel::WARNING:
            return node->awarn.history().str();
        case ErrorState::ErrorLevel::NONE:
            return node->ainfo.history().str();
        }
    }
    return {};
}

bool NodeFacadeRemote::hasParameter(const std::string &name) const
{
    if(auto node = nh_->getNode().lock()){
        return node->hasParameter(name);
    }
    throw std::runtime_error("tried to check a parameter from an invalid node");
}

//template <typename T>
//T NodeFacadeRemote::readParameter(const std::string& name) const
//{
//    if(auto node = nh_->getNode().lock()){
//        return node->readParameter<T>(name);
//    }
//    throw std::runtime_error("tried to read a parameter from an invalid node");
//}

//template <typename T>
//void NodeFacadeRemote::setParameter(const std::string& name, const T& value)
//{
//    if(auto node = nh_->getNode().lock()){
//        node->setParameter<T>(name, value);
//    } else {
//        throw std::runtime_error("tried to set a parameter from an invalid node");
//    }
//}



//template CSAPEX_EXPORT bool NodeFacadeRemote::readParameter<bool>(const std::string& name) const;
//template CSAPEX_EXPORT double NodeFacadeRemote::readParameter<double>(const std::string& name) const;
//template CSAPEX_EXPORT int NodeFacadeRemote::readParameter<int>(const std::string& name) const;
//template CSAPEX_EXPORT std::string NodeFacadeRemote::readParameter<std::string>(const std::string& name) const;
//template CSAPEX_EXPORT std::pair<int,int> NodeFacadeRemote::readParameter<std::pair<int,int> >(const std::string& name) const;
//template CSAPEX_EXPORT std::pair<double,double> NodeFacadeRemote::readParameter<std::pair<double,double> >(const std::string& name) const;
//template CSAPEX_EXPORT std::pair<std::string, bool> NodeFacadeRemote::readParameter<std::pair<std::string, bool> >(const std::string& name) const;
//template CSAPEX_EXPORT std::vector<double> NodeFacadeRemote::readParameter<std::vector<double> >(const std::string& name) const;
//template CSAPEX_EXPORT std::vector<int> NodeFacadeRemote::readParameter<std::vector<int> >(const std::string& name) const;


//template CSAPEX_EXPORT void NodeFacadeRemote::setParameter<bool>(const std::string& name, const bool& value);
//template CSAPEX_EXPORT void NodeFacadeRemote::setParameter<double>(const std::string& name, const double& value);
//template CSAPEX_EXPORT void NodeFacadeRemote::setParameter<int>(const std::string& name, const int& value);
//template CSAPEX_EXPORT void NodeFacadeRemote::setParameter<std::string>(const std::string& name, const std::string& value);
//template CSAPEX_EXPORT void NodeFacadeRemote::setParameter<std::pair<int,int> > (const std::string& name, const std::pair<int,int>& value);
//template CSAPEX_EXPORT void NodeFacadeRemote::setParameter<std::pair<double,double> >(const std::string& name, const std::pair<double,double>& value);
//template CSAPEX_EXPORT void NodeFacadeRemote::setParameter<std::pair<std::string, bool> >(const std::string& name, const std::pair<std::string, bool>& value);
//template CSAPEX_EXPORT void NodeFacadeRemote::setParameter<std::vector<int> >(const std::string& name, const std::vector<int>& value);
//template CSAPEX_EXPORT void NodeFacadeRemote::setParameter<std::vector<double> >(const std::string& name, const std::vector<double>& value);

#endif
