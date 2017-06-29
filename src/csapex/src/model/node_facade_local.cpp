/// HEADER
#include <csapex/model/node_facade_local.h>

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

NodeFacadeLocal::NodeFacadeLocal(NodeHandlePtr nh, NodeWorkerPtr nw, NodeRunnerPtr nr)
    : nh_(nh), nw_(nw), nr_(nr)
{
    nh->setNodeRunner(nr_);

    connectNodeHandle();
    connectNodeWorker();
}
NodeFacadeLocal::NodeFacadeLocal(NodeHandlePtr nh)
    : nh_(nh)
{
    connectNodeHandle();
}


void NodeFacadeLocal::connectNodeHandle()
{
    observe(nh_->connector_created, connector_created);
    observe(nh_->connector_removed, connector_removed);
    observe(nh_->node_state_changed, node_state_changed);


    observe(nh_->connection_in_prograss, connection_in_prograss);
    observe(nh_->connection_done, connection_done);
    observe(nh_->connection_start, connection_start);


    observe(nh_->parameters_changed, parameters_changed);
}

void NodeFacadeLocal::connectNodeWorker()
{
    observe(nw_->start_profiling, [this](NodeWorker*) {
        start_profiling(this);
    });
    observe(nw_->stop_profiling,  [this](NodeWorker*) {
        stop_profiling(this);
    });

    observe(nw_->destroyed, destroyed);
    observe(nw_->notification, notification);

    observe(nw_->messages_processed, messages_processed);

    observe(nw_->interval_start, [this](NodeWorker*, ActivityType type, std::shared_ptr<const Interval> stamp) {
        interval_start(this, type, stamp);
    });
    observe(nw_->interval_end,  [this](NodeWorker*, std::shared_ptr<const Interval> stamp) {
        interval_end(this, stamp);
    });
}

NodeFacadeLocal::~NodeFacadeLocal()
{
}

std::string NodeFacadeLocal::getType() const
{
    return nh_->getType();
}

UUID NodeFacadeLocal::getUUID() const
{
    return nh_->getUUID();
}

bool NodeFacadeLocal::isActive() const
{
    return nh_->isActive();
}

void NodeFacadeLocal::setActive(bool active)
{
    nh_->setActive(active);
}

bool NodeFacadeLocal::isProcessingEnabled() const
{
    if(nw_) {
        return nw_->isProcessingEnabled();
    } else {
        return false;
    }
}

bool NodeFacadeLocal::isGraph() const
{
    return nh_->isGraph();
}

AUUID NodeFacadeLocal::getSubgraphAUUID() const
{
    return nh_->getSubgraphAUUID();
}

bool NodeFacadeLocal::isSource() const
{
    return nh_->isSource();
}
bool NodeFacadeLocal::isSink() const
{
    return nh_->isSink();
}

bool NodeFacadeLocal::isProcessingNothingMessages() const
{
    return getNode()->processMessageMarkers();
}

bool NodeFacadeLocal::isParameterInput(const UUID& id)
{
    return nh_->isParameterInput(id);
}

bool NodeFacadeLocal::isParameterOutput(const UUID& id)
{
    return nh_->isParameterOutput(id);
}

bool NodeFacadeLocal::isVariadic() const
{
    return nh_->isVariadic();
}
bool NodeFacadeLocal::hasVariadicInputs() const
{
    return nh_->hasVariadicInputs();
}
bool NodeFacadeLocal::hasVariadicOutputs() const
{
    return nh_->hasVariadicOutputs();
}
bool NodeFacadeLocal::hasVariadicEvents() const
{
    return nh_->hasVariadicEvents();
}
bool NodeFacadeLocal::hasVariadicSlots() const
{
    return nh_->hasVariadicSlots();
}


std::vector<ConnectorDescription> NodeFacadeLocal::getInputs() const
{
    return nh_->getExternalInputDescriptions();
}

std::vector<ConnectorDescription> NodeFacadeLocal::getOutputs() const
{
    return nh_->getExternalOutputDescriptions();
}

std::vector<ConnectorDescription> NodeFacadeLocal::getEvents() const
{
    return nh_->getExternalEventDescriptions();
}

std::vector<ConnectorDescription> NodeFacadeLocal::getSlots() const
{
    return nh_->getExternalSlotDescriptions();
}


std::vector<ConnectorDescription> NodeFacadeLocal::getInternalInputs() const
{
    return nh_->getInternalInputDescriptions();
}

std::vector<ConnectorDescription> NodeFacadeLocal::getInternalOutputs() const
{
    return nh_->getInternalOutputDescriptions();
}

std::vector<ConnectorDescription> NodeFacadeLocal::getInternalEvents() const
{
    return nh_->getInternalEventDescriptions();
}

std::vector<ConnectorDescription> NodeFacadeLocal::getInternalSlots() const
{
    return nh_->getInternalSlotDescriptions();
}


NodeCharacteristics NodeFacadeLocal::getNodeCharacteristics() const
{
    return nh_->getVertex()->getNodeCharacteristics();
}



std::vector<param::ParameterPtr> NodeFacadeLocal::getParameters() const
{
    return getNode()->getParameters();
}

param::ParameterPtr NodeFacadeLocal::getParameter(const std::string &name) const
{
    return getNode()->getParameter(name);
}

bool NodeFacadeLocal::isProfiling() const
{
    if(nw_) {
        return nw_->isProfiling();
    } else {
        return false;
    }
}
void NodeFacadeLocal::setProfiling(bool profiling)
{
    if(nw_) {
        nw_->setProfiling(profiling);
    }
}
bool NodeFacadeLocal::isError() const
{
    if(nw_) {
        return nw_->isError();
    } else {
        return false;
    }
}
ErrorState::ErrorLevel NodeFacadeLocal::errorLevel() const
{
    if(nw_) {
        return nw_->errorLevel();
    } else {
        return ErrorState::ErrorLevel::NONE;
    }
}
std::string NodeFacadeLocal::errorMessage() const
{
    if(nw_) {
        return nw_->errorMessage();
    } else {
        return {};
    }
}

ExecutionState NodeFacadeLocal::getExecutionState() const
{
    if(nw_) {
        return nw_->getExecutionState();
    } else {
        return ExecutionState::UNKNOWN;
    }
}


std::string NodeFacadeLocal::getLabel() const
{
    return nh_->getNodeState()->getLabel();
}

double NodeFacadeLocal::getExecutionFrequency() const
{
    return nh_->getRate().getEffectiveFrequency();
}

double NodeFacadeLocal::getMaximumFrequency() const
{
    return nh_->getNodeState()->getMaximumFrequency();
}

NodeHandlePtr NodeFacadeLocal::getNodeHandle() const
{
    return nh_;
}

NodePtr NodeFacadeLocal::getNode() const
{
    return nh_->getNode().lock();
}

NodeWorkerPtr NodeFacadeLocal::getNodeWorker() const
{
    return nw_;
}

NodeRunnerPtr NodeFacadeLocal::getNodeRunner() const
{
    return nr_;
}

NodeStatePtr NodeFacadeLocal::getNodeState() const
{
    return nh_->getNodeState();
}
NodeStatePtr NodeFacadeLocal::getNodeStateCopy() const
{
    return nh_->getNodeStateCopy();
}

void NodeFacadeLocal::setNodeState(NodeStatePtr memento)
{
    nh_->setNodeState(memento);
}

GenericStateConstPtr NodeFacadeLocal::getParameterState() const
{
    return nh_->getNode().lock()->getParameterStateClone();
}

ProfilerPtr NodeFacadeLocal::getProfiler()
{
    if(nw_) {
        return nw_->getProfiler();
    } else {
        return {};
    }
}

std::string NodeFacadeLocal::getDebugDescription() const
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

std::string NodeFacadeLocal::getLoggerOutput(ErrorState::ErrorLevel level) const
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

bool NodeFacadeLocal::hasParameter(const std::string &name) const
{
    if(auto node = nh_->getNode().lock()){
        return node->hasParameter(name);
    }
    throw std::runtime_error("tried to check a parameter from an invalid node");
}

//template <typename T>
//T NodeFacadeLocal::readParameter(const std::string& name) const
//{
//    if(auto node = nh_->getNode().lock()){
//        return node->readParameter<T>(name);
//    }
//    throw std::runtime_error("tried to read a parameter from an invalid node");
//}

//template <typename T>
//void NodeFacadeLocal::setParameter(const std::string& name, const T& value)
//{
//    if(auto node = nh_->getNode().lock()){
//        node->setParameter<T>(name, value);
//    } else {
//        throw std::runtime_error("tried to set a parameter from an invalid node");
//    }
//}



//template CSAPEX_EXPORT bool NodeFacadeLocal::readParameter<bool>(const std::string& name) const;
//template CSAPEX_EXPORT double NodeFacadeLocal::readParameter<double>(const std::string& name) const;
//template CSAPEX_EXPORT int NodeFacadeLocal::readParameter<int>(const std::string& name) const;
//template CSAPEX_EXPORT std::string NodeFacadeLocal::readParameter<std::string>(const std::string& name) const;
//template CSAPEX_EXPORT std::pair<int,int> NodeFacadeLocal::readParameter<std::pair<int,int> >(const std::string& name) const;
//template CSAPEX_EXPORT std::pair<double,double> NodeFacadeLocal::readParameter<std::pair<double,double> >(const std::string& name) const;
//template CSAPEX_EXPORT std::pair<std::string, bool> NodeFacadeLocal::readParameter<std::pair<std::string, bool> >(const std::string& name) const;
//template CSAPEX_EXPORT std::vector<double> NodeFacadeLocal::readParameter<std::vector<double> >(const std::string& name) const;
//template CSAPEX_EXPORT std::vector<int> NodeFacadeLocal::readParameter<std::vector<int> >(const std::string& name) const;


//template CSAPEX_EXPORT void NodeFacadeLocal::setParameter<bool>(const std::string& name, const bool& value);
//template CSAPEX_EXPORT void NodeFacadeLocal::setParameter<double>(const std::string& name, const double& value);
//template CSAPEX_EXPORT void NodeFacadeLocal::setParameter<int>(const std::string& name, const int& value);
//template CSAPEX_EXPORT void NodeFacadeLocal::setParameter<std::string>(const std::string& name, const std::string& value);
//template CSAPEX_EXPORT void NodeFacadeLocal::setParameter<std::pair<int,int> > (const std::string& name, const std::pair<int,int>& value);
//template CSAPEX_EXPORT void NodeFacadeLocal::setParameter<std::pair<double,double> >(const std::string& name, const std::pair<double,double>& value);
//template CSAPEX_EXPORT void NodeFacadeLocal::setParameter<std::pair<std::string, bool> >(const std::string& name, const std::pair<std::string, bool>& value);
//template CSAPEX_EXPORT void NodeFacadeLocal::setParameter<std::vector<int> >(const std::string& name, const std::vector<int>& value);
//template CSAPEX_EXPORT void NodeFacadeLocal::setParameter<std::vector<double> >(const std::string& name, const std::vector<double>& value);

