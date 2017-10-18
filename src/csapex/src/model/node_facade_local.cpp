/// HEADER
#include <csapex/model/node_facade_local.h>

/// PROJECT
#include <csapex/model/node_handle.h>
#include <csapex/model/node_worker.h>
#include <csapex/model/node_state.h>
#include <csapex/msg/input_transition.h>
#include <csapex/msg/output_transition.h>
#include <csapex/signal/event.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/model/graph/vertex.h>
#include <csapex/model/node.h>
#include <csapex/profiling/profiler_local.h>

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
    observe(nh_->connector_created, [this](ConnectablePtr c, bool internal) {
        connector_created(c->getDescription());
        if(internal) {
            triggerInternalConnectorsChanged(c);
        } else {
            triggerExternalConnectorsChanged(c);
        }
    });
    observe(nh_->connector_removed, [this](ConnectablePtr c, bool internal) {
        connector_removed(c->getDescription());
        if(internal) {
            triggerInternalConnectorsChanged(c);
        } else {
            triggerExternalConnectorsChanged(c);
        }
    });


    observe(nh_->node_state_changed, node_state_changed);
    observe(nh_->activation_changed, activation_changed);


    observe(nh_->connection_done, [this](ConnectablePtr c) { connection_done(c->getDescription()); });
    observe(nh_->connection_start, [this](ConnectablePtr c) { connection_start(c->getDescription()); });


    observe(nh_->parameters_changed, parameters_changed);

    NodeStatePtr state = nh_->getNodeState();

    observe(*(state->label_changed), [this]() {
        label_changed(getLabel());
    });
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

    observe(nw_->execution_state_changed, [this]() {
        execution_state_changed(nw_->getExecutionState());
    });

    observe(nw_->interval_start, [this](NodeWorker*, ActivityType type, std::shared_ptr<const Interval> stamp) {
        interval_start(this, type, stamp);
    });
    observe(nw_->interval_end,  [this](NodeWorker*, std::shared_ptr<const Interval> stamp) {
        interval_end(this, stamp);
    });

    observe(nw_->error_event, [this](bool error, const std::string& msg, ErrorLevel level) {
        setError(error, msg, level);
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

AUUID NodeFacadeLocal::getAUUID() const
{
    return nh_->getUUID().getAbsoluteUUID();
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



std::vector<ConnectorDescription> NodeFacadeLocal::getExternalInputs() const
{
    return nh_->getExternalInputDescriptions();
}

std::vector<ConnectorDescription> NodeFacadeLocal::getExternalOutputs() const
{
    return nh_->getExternalOutputDescriptions();
}

std::vector<ConnectorDescription> NodeFacadeLocal::getExternalEvents() const
{
    return nh_->getExternalEventDescriptions();
}

std::vector<ConnectorDescription> NodeFacadeLocal::getExternalSlots() const
{
    return nh_->getExternalSlotDescriptions();
}

ConnectorPtr NodeFacadeLocal::getConnector(const UUID &id) const
{
    return nh_->getConnector(id);
}

ConnectorPtr NodeFacadeLocal::getConnectorNoThrow(const UUID& id) const noexcept
{
    return nh_->getConnectorNoThrow(id);
}

NodeCharacteristics NodeFacadeLocal::getNodeCharacteristics() const
{
    return nh_->getVertex()->getNodeCharacteristics();
}


ConnectorPtr NodeFacadeLocal::getParameterInput(const std::string& name) const
{
    return nh_->getParameterInput(name).lock();
}
ConnectorPtr NodeFacadeLocal::getParameterOutput(const std::string& name) const
{
    return nh_->getParameterOutput(name).lock();
}



std::vector<param::ParameterPtr> NodeFacadeLocal::getParameters() const
{
    return getNode()->getParameters();
}

param::ParameterPtr NodeFacadeLocal::getParameter(const std::string &name) const
{
    return getNode()->getParameter(name);
}

bool NodeFacadeLocal::canStartStepping() const
{
    if(!nr_) {
        return false;
    }
    return nr_->canStartStepping();
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
    ss << (events_enabled ? "enabled" : "disabled") << ", ";
    ss << (canStartStepping() ? "canStartStepping" : "!canStartStepping");
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

void NodeFacadeLocal::triggerExternalConnectorsChanged(const ConnectableConstPtr& connector)
{
    if(connector->isInput()) {
        if(connector->isSynchronous()) {
            external_inputs_changed(nh_->getExternalInputDescriptions());
        } else {
            external_slots_changed(nh_->getExternalSlotDescriptions());
        }

    } else {
        if(connector->isSynchronous()) {
            external_outputs_changed(nh_->getExternalOutputDescriptions());
        } else {
            external_events_changed(nh_->getExternalEventDescriptions());
        }
    }
}

void NodeFacadeLocal::triggerInternalConnectorsChanged(const ConnectableConstPtr& connector)
{
    if(connector->isInput()) {
        if(connector->isSynchronous()) {
            internal_inputs_changed(nh_->getExternalInputDescriptions());
        } else {
            internal_slots_changed(nh_->getExternalSlotDescriptions());
        }

    } else {
        if(connector->isSynchronous()) {
            internal_outputs_changed(nh_->getExternalOutputDescriptions());
        } else {
            internal_events_changed(nh_->getExternalEventDescriptions());
        }
    }
}
