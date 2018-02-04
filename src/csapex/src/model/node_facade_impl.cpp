/// HEADER
#include <csapex/model/node_facade_impl.h>

/// PROJECT
#include <csapex/model/generic_state.h>
#include <csapex/model/graph/graph_impl.h>
#include <csapex/model/graph/vertex.h>
#include <csapex/model/node.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_state.h>
#include <csapex/model/node_worker.h>
#include <csapex/model/subgraph_node.h>
#include <csapex/msg/input.h>
#include <csapex/msg/input_transition.h>
#include <csapex/msg/output.h>
#include <csapex/msg/output_transition.h>
#include <csapex/profiling/profiler_impl.h>
#include <csapex/scheduling/scheduler.h>
#include <csapex/signal/event.h>

/// SYSTEM
#include <iostream>
#include <sstream>

using namespace csapex;

NodeFacadeImplementation::NodeFacadeImplementation(NodeHandlePtr nh, NodeWorkerPtr nw, NodeRunnerPtr nr)
    : nh_(nh), nw_(nw), nr_(nr)
{
    nh->setNodeRunner(nr_);

    setNodeWorker(nw);

    connectNodeHandle();
    connectNodeRunner();
}
NodeFacadeImplementation::NodeFacadeImplementation(NodeHandlePtr nh)
    : nh_(nh)
{
    connectNodeHandle();
}


void NodeFacadeImplementation::connectNodeHandle()
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


    observe(nh_->node_state_changed, [this]() {
        NodeStatePtr state = nh_->getNodeState();
        node_state_changed(state);
    });
    observe(nh_->activation_changed, activation_changed);


    observe(nh_->connection_added, [this](ConnectablePtr c) {
        connection_added(c->getDescription());
        triggerExternalConnectorsChanged(c);
    });
    observe(nh_->connection_removed, [this](ConnectablePtr c) {
        connection_removed(c->getDescription());
        triggerExternalConnectorsChanged(c);
    });
    observe(nh_->connection_start, [this](ConnectablePtr c) {
        connection_start(c->getDescription());
    });


    observe(nh_->parameters_changed, parameters_changed);

    observe(nh_->raw_data_connection, raw_data_connection);


    NodeStatePtr state = nh_->getNodeState();

    observe(*(state->label_changed), [this]() {
        label_changed(getLabel());
    });

    observe(*(state->thread_changed), [this]() {
        scheduler_changed(getSchedulerId());
    });

    GenericStatePtr paramstate = state->getParameterState();

    if(paramstate) {
        observe(*(paramstate->parameter_added), [this](const param::ParameterPtr& p) {
            parameter_added(p);
        });
        observe(*(paramstate->parameter_changed), [this](const param::Parameter* p) {
            if(parameter_changed.isConnected()) {
                NodeStatePtr state = nh_->getNodeState();
                GenericStatePtr paramstate = state->getParameterState();
                parameter_changed(paramstate->getParameter(p->name()));
            }
        });
        observe(*(paramstate->parameter_removed), [this](const param::ParameterPtr& p) {
            parameter_removed(p);
        });

        observe(*(paramstate->parameter_set_changed), [this]() {
            parameter_set_changed();
        });
    }
}

void NodeFacadeImplementation::connectNodeWorker()
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

    observe(nw_->error_event, [this](bool error, const std::string& msg, ErrorLevel level) {
        setError(error, msg, level);
    });
}

void NodeFacadeImplementation::connectNodeRunner()
{
    observe(nr_->notification, notification);
}

NodeFacadeImplementation::~NodeFacadeImplementation()
{
}

std::string NodeFacadeImplementation::getType() const
{
    return nh_->getType();
}

UUID NodeFacadeImplementation::getUUID() const
{
    return nh_->getUUID();
}

AUUID NodeFacadeImplementation::getAUUID() const
{
    return nh_->getUUID().getAbsoluteUUID();
}

bool NodeFacadeImplementation::isActive() const
{
    return nh_->isActive();
}

void NodeFacadeImplementation::setActive(bool active)
{
    nh_->setActive(active);
}

bool NodeFacadeImplementation::isProcessingEnabled() const
{
    if(nw_) {
        return nw_->isProcessingEnabled();
    } else {
        return false;
    }
}

bool NodeFacadeImplementation::isGraph() const
{
    return nh_->isGraph();
}

AUUID NodeFacadeImplementation::getSubgraphAUUID() const
{
    return nh_->getSubgraphAUUID();
}

GraphPtr NodeFacadeImplementation::getSubgraph() const
{
    return std::dynamic_pointer_cast<SubgraphNode>(nh_->getNode().lock())->getLocalGraph();
}

bool NodeFacadeImplementation::isSource() const
{
    return nh_->isSource();
}
bool NodeFacadeImplementation::isSink() const
{
    return nh_->isSink();
}

bool NodeFacadeImplementation::isProcessingNothingMessages() const
{
    return getNode()->processNothingMarkers();
}

bool NodeFacadeImplementation::isParameterInput(const UUID& id)
{
    return nh_->isParameterInput(id);
}

bool NodeFacadeImplementation::isParameterOutput(const UUID& id)
{
    return nh_->isParameterOutput(id);
}

bool NodeFacadeImplementation::isVariadic() const
{
    return nh_->isVariadic();
}
bool NodeFacadeImplementation::hasVariadicInputs() const
{
    return nh_->hasVariadicInputs();
}
bool NodeFacadeImplementation::hasVariadicOutputs() const
{
    return nh_->hasVariadicOutputs();
}
bool NodeFacadeImplementation::hasVariadicEvents() const
{
    return nh_->hasVariadicEvents();
}
bool NodeFacadeImplementation::hasVariadicSlots() const
{
    return nh_->hasVariadicSlots();
}


std::vector<ConnectorDescription> NodeFacadeImplementation::getInternalInputs() const
{
    return nh_->getInternalInputDescriptions();
}

std::vector<ConnectorDescription> NodeFacadeImplementation::getInternalOutputs() const
{
    return nh_->getInternalOutputDescriptions();
}

std::vector<ConnectorDescription> NodeFacadeImplementation::getInternalEvents() const
{
    return nh_->getInternalEventDescriptions();
}

std::vector<ConnectorDescription> NodeFacadeImplementation::getInternalSlots() const
{
    return nh_->getInternalSlotDescriptions();
}



std::vector<ConnectorDescription> NodeFacadeImplementation::getExternalInputs() const
{
    return nh_->getExternalInputDescriptions();
}

std::vector<ConnectorDescription> NodeFacadeImplementation::getExternalOutputs() const
{
    return nh_->getExternalOutputDescriptions();
}

std::vector<ConnectorDescription> NodeFacadeImplementation::getExternalEvents() const
{
    return nh_->getExternalEventDescriptions();
}

std::vector<ConnectorDescription> NodeFacadeImplementation::getExternalSlots() const
{
    return nh_->getExternalSlotDescriptions();
}

ConnectorPtr NodeFacadeImplementation::getConnector(const UUID &id) const
{
    return nh_->getConnector(id);
}

ConnectorPtr NodeFacadeImplementation::getConnectorNoThrow(const UUID& id) const noexcept
{
    return nh_->getConnectorNoThrow(id);
}

NodeCharacteristics NodeFacadeImplementation::getNodeCharacteristics() const
{
    return nh_->getVertex()->getNodeCharacteristics();
}


ConnectorPtr NodeFacadeImplementation::getParameterInput(const std::string& name) const
{
    return nh_->getParameterInput(name).lock();
}
ConnectorPtr NodeFacadeImplementation::getParameterOutput(const std::string& name) const
{
    return nh_->getParameterOutput(name).lock();
}



std::vector<param::ParameterPtr> NodeFacadeImplementation::getParameters() const
{
    return getNode()->getParameters();
}

param::ParameterPtr NodeFacadeImplementation::getParameter(const std::string &name) const
{
    return getNode()->getParameter(name);
}

bool NodeFacadeImplementation::canStartStepping() const
{
    if(!nr_) {
        return false;
    }
    return nr_->canStartStepping();
}

bool NodeFacadeImplementation::canProcess() const
{
    if(nw_) {
        return nw_->canProcess();
    } else {
        return false;
    }
}
bool NodeFacadeImplementation::isProcessing() const
{
    if(nw_) {
        return nw_->isProcessing();
    } else {
        return false;
    }
}
bool NodeFacadeImplementation::startProcessingMessages()
{
    if(nw_) {
        return nw_->startProcessingMessages();
    } else {
        return false;
    }
}

bool NodeFacadeImplementation::isProfiling() const
{
    if(nw_) {
        return nw_->isProfiling();
    } else {
        return false;
    }
}
void NodeFacadeImplementation::setProfiling(bool profiling)
{
    if(nw_) {
        nw_->setProfiling(profiling);
    }
}

ExecutionState NodeFacadeImplementation::getExecutionState() const
{
    if(nw_) {
        return nw_->getExecutionState();
    } else {
        return ExecutionState::UNKNOWN;
    }
}


std::string NodeFacadeImplementation::getLabel() const
{
    return nh_->getNodeState()->getLabel();
}

int NodeFacadeImplementation::getSchedulerId() const
{
    if(!nr_) {
        return -1;
    }
    if(auto scheduler = nr_->getScheduler()) {
        return scheduler->id();
    } else {
        return -1;
    }
}

double NodeFacadeImplementation::getExecutionFrequency() const
{
    return nh_->getRate().getEffectiveFrequency();
}

double NodeFacadeImplementation::getMaximumFrequency() const
{
    return nh_->getNodeState()->getMaximumFrequency();
}

NodeHandlePtr NodeFacadeImplementation::getNodeHandle() const
{
    return nh_;
}

NodePtr NodeFacadeImplementation::getNode() const
{
    return nh_->getNode().lock();
}

NodeRunnerPtr NodeFacadeImplementation::getNodeRunner() const
{
    return nr_;
}

void NodeFacadeImplementation::setNodeWorker(NodeWorkerPtr worker)
{
    nw_ = worker;
    connectNodeWorker();

    nh_->setNodeWorker(worker.get());
    nr_->setNodeWorker(worker);
}


void NodeFacadeImplementation::replaceNodeWorker(NodeWorkerPtr worker)
{
    setNodeWorker(worker);

    worker->initialize();
}

NodeStatePtr NodeFacadeImplementation::getNodeState() const
{
    return nh_->getNodeState();
}
NodeStatePtr NodeFacadeImplementation::getNodeStateCopy() const
{
    return nh_->getNodeStateCopy();
}

void NodeFacadeImplementation::setNodeState(NodeStatePtr memento)
{
    nh_->setNodeState(memento);
}

ProfilerPtr NodeFacadeImplementation::getProfiler()
{
    if(nw_) {
        return nw_->getProfiler();
    } else {
        return {};
    }
}

std::string NodeFacadeImplementation::getDebugDescription() const
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

std::string NodeFacadeImplementation::getLoggerOutput(ErrorState::ErrorLevel level) const
{
    if(NodePtr node = nh_->getNode().lock()){
        switch(level) {
        case ErrorState::ErrorLevel::ERROR:
            return node->aerr.history().str();
        case ErrorState::ErrorLevel::WARNING:
            return node->awarn.history().str();
        case ErrorState::ErrorLevel::INFO:
            return node->ainfo.history().str();
        case ErrorState::ErrorLevel::NONE:
            return node->ainfo.history().str();
        }
    }
    return {};
}

bool NodeFacadeImplementation::hasParameter(const std::string &name) const
{
    if(auto node = nh_->getNode().lock()){
        return node->hasParameter(name);
    }
    throw std::runtime_error("tried to check a parameter from an invalid node");
}

void NodeFacadeImplementation::triggerExternalConnectorsChanged(const ConnectableConstPtr& connector)
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

void NodeFacadeImplementation::triggerInternalConnectorsChanged(const ConnectableConstPtr& connector)
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
