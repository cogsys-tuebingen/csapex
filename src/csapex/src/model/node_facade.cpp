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
#include <csapex/model/node.h>

/// SYSTEM
#include <iostream>
#include <sstream>

using namespace csapex;

NodeFacade::NodeFacade(NodeHandlePtr nh, NodeWorkerPtr nw, NodeRunnerPtr nr)
    : NodeFacade(nh)
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

NodeFacade::NodeFacade(NodeHandlePtr nh)
    : nh_(nh)
{
    observe(nh->connector_created, connector_created);
    observe(nh->connector_removed, connector_removed);
    observe(nh->node_state_changed, node_state_changed);


    observe(nh->connection_in_prograss, connection_in_prograss);
    observe(nh->connection_done, connection_done);
    observe(nh->connection_start, connection_start);


    observe(nh->parameters_changed, parameters_changed);
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

void NodeFacade::setActive(bool active)
{
    nh_->setActive(active);
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

bool NodeFacade::isParameterInput(const UUID& id)
{
    return nh_->isParameterInput(id);
}

bool NodeFacade::isParameterOutput(const UUID& id)
{
    return nh_->isParameterOutput(id);
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


std::vector<ConnectorDescription> NodeFacade::getInputs() const
{
    return nh_->getExternalInputDescriptions();
}

std::vector<ConnectorDescription> NodeFacade::getOutputs() const
{
    return nh_->getExternalOutputDescriptions();
}

std::vector<ConnectorDescription> NodeFacade::getEvents() const
{
    return nh_->getExternalEventDescriptions();
}

std::vector<ConnectorDescription> NodeFacade::getSlots() const
{
    return nh_->getExternalSlotDescriptions();
}


std::vector<ConnectorDescription> NodeFacade::getInternalInputs() const
{
    return nh_->getInternalInputDescriptions();
}

std::vector<ConnectorDescription> NodeFacade::getInternalOutputs() const
{
    return nh_->getInternalOutputDescriptions();
}

std::vector<ConnectorDescription> NodeFacade::getInternalEvents() const
{
    return nh_->getInternalEventDescriptions();
}

std::vector<ConnectorDescription> NodeFacade::getInternalSlots() const
{
    return nh_->getInternalSlotDescriptions();
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

NodeHandlePtr NodeFacade::getNodeHandle()
{
    return nh_;
}

NodePtr NodeFacade::getNode()
{
    return nh_->getNode().lock();
}

NodeWorkerPtr NodeFacade::getNodeWorker()
{
    return nw_;
}

NodeRunnerPtr NodeFacade::getNodeRunner()
{
    return nr_;
}

NodeStatePtr NodeFacade::getNodeState() const
{
    return nh_->getNodeState();
}
NodeStatePtr NodeFacade::getNodeStateCopy() const
{
    return nh_->getNodeStateCopy();
}

void NodeFacade::setNodeState(NodeStatePtr memento)
{
    nh_->setNodeState(memento);
}

GenericStateConstPtr NodeFacade::getParameterState() const
{
    return nh_->getNode().lock()->getParameterStateClone();
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

std::string NodeFacade::getLoggerOutput(ErrorState::ErrorLevel level) const
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

bool NodeFacade::hasParameter(const std::string &name) const
{
    if(auto node = nh_->getNode().lock()){
        return node->hasParameter(name);
    }
    throw std::runtime_error("tried to check a parameter from an invalid node");
}

template <typename T>
T NodeFacade::readParameter(const std::string& name) const
{
    if(auto node = nh_->getNode().lock()){
        return node->readParameter<T>(name);
    }
    throw std::runtime_error("tried to read a parameter from an invalid node");
}

template <typename T>
void NodeFacade::setParameter(const std::string& name, const T& value)
{
    if(auto node = nh_->getNode().lock()){
        node->setParameter<T>(name, value);
    } else {
        throw std::runtime_error("tried to set a parameter from an invalid node");
    }
}



template CSAPEX_EXPORT bool NodeFacade::readParameter<bool>(const std::string& name) const;
template CSAPEX_EXPORT double NodeFacade::readParameter<double>(const std::string& name) const;
template CSAPEX_EXPORT int NodeFacade::readParameter<int>(const std::string& name) const;
template CSAPEX_EXPORT std::string NodeFacade::readParameter<std::string>(const std::string& name) const;
template CSAPEX_EXPORT std::pair<int,int> NodeFacade::readParameter<std::pair<int,int> >(const std::string& name) const;
template CSAPEX_EXPORT std::pair<double,double> NodeFacade::readParameter<std::pair<double,double> >(const std::string& name) const;
template CSAPEX_EXPORT std::pair<std::string, bool> NodeFacade::readParameter<std::pair<std::string, bool> >(const std::string& name) const;
template CSAPEX_EXPORT std::vector<double> NodeFacade::readParameter<std::vector<double> >(const std::string& name) const;
template CSAPEX_EXPORT std::vector<int> NodeFacade::readParameter<std::vector<int> >(const std::string& name) const;


template CSAPEX_EXPORT void NodeFacade::setParameter<bool>(const std::string& name, const bool& value);
template CSAPEX_EXPORT void NodeFacade::setParameter<double>(const std::string& name, const double& value);
template CSAPEX_EXPORT void NodeFacade::setParameter<int>(const std::string& name, const int& value);
template CSAPEX_EXPORT void NodeFacade::setParameter<std::string>(const std::string& name, const std::string& value);
template CSAPEX_EXPORT void NodeFacade::setParameter<std::pair<int,int> > (const std::string& name, const std::pair<int,int>& value);
template CSAPEX_EXPORT void NodeFacade::setParameter<std::pair<double,double> >(const std::string& name, const std::pair<double,double>& value);
template CSAPEX_EXPORT void NodeFacade::setParameter<std::pair<std::string, bool> >(const std::string& name, const std::pair<std::string, bool>& value);
template CSAPEX_EXPORT void NodeFacade::setParameter<std::vector<int> >(const std::string& name, const std::vector<int>& value);
template CSAPEX_EXPORT void NodeFacade::setParameter<std::vector<double> >(const std::string& name, const std::vector<double>& value);

