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
#include <csapex/io/session.h>
#include <csapex/io/protcol/node_requests.h>
#include <csapex/io/raw_message.h>
#include <csapex/model/connector_remote.h>

/// SYSTEM
#include <iostream>
#include <sstream>

using namespace csapex;
using namespace csapex;

NodeFacadeRemote::NodeFacadeRemote(SessionPtr session,
                                   NodeHandlePtr nh, NodeWorkerPtr nw, NodeRunnerPtr nr)
    : session_(session),
      nh_(nh), nw_(nw), nr_(nr)
{
    nh->setNodeRunner(nr_);

    connectNodeHandle();
    connectNodeWorker();


    observe(remote_data_connection.first_connected, [this]() {
        session_->sendRequest<NodeRequests>(NodeRequests::NodeRequestType::AddClient, getUUID().getAbsoluteUUID());
    });

    observe(remote_data_connection.last_disconnected, [this]() {
        session_->sendRequest<NodeRequests>(NodeRequests::NodeRequestType::RemoveClient, getUUID().getAbsoluteUUID());
    });

    observe(session->raw_packet_received(getUUID().getAbsoluteUUID()), [this](const RawMessageConstPtr& data) {
        remote_data_connection(data);
    });
}

NodeFacadeRemote::~NodeFacadeRemote()
{
}

void NodeFacadeRemote::connectNodeHandle()
{
    observe(nh_->connector_created, [this](ConnectorPtr connector) {
        createConnectorProxy(connector);
    });
    for(ConnectorPtr c : nh_->getExternalConnectors()) {
        createConnectorProxy(c);
    }


    observe(nh_->connector_removed, connector_removed);
    observe(nh_->node_state_changed, node_state_changed);


    observe(nh_->connection_in_prograss, connection_in_prograss);
    observe(nh_->connection_done, connection_done);
    observe(nh_->connection_start, connection_start);


    observe(nh_->parameters_changed, parameters_changed);
}

void NodeFacadeRemote::connectNodeWorker()
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

void NodeFacadeRemote::createConnectorProxy(const ConnectorPtr &connector)
{
    UUID uuid = connector->getUUID();
    ConnectableOwnerPtr owner;
    std::shared_ptr<ConnectorRemote> proxy = std::make_shared<ConnectorRemote>(uuid, owner, session_, connector);
    remote_connectors_[uuid] = proxy;
    connector_created(proxy);
}

std::string NodeFacadeRemote::getType() const
{
    return nh_->getType();
}

UUID NodeFacadeRemote::getUUID() const
{
    return nh_->getUUID();
}

AUUID NodeFacadeRemote::getAUUID() const
{
    return nh_->getUUID().getAbsoluteUUID();
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

bool NodeFacadeRemote::isProcessingNothingMessages() const
{
    return nh_->getNode().lock()->processMessageMarkers();
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



std::vector<ConnectorDescription> NodeFacadeRemote::getExternalInputs() const
{
    return nh_->getExternalInputDescriptions();
}

std::vector<ConnectorDescription> NodeFacadeRemote::getExternalOutputs() const
{
    return nh_->getExternalOutputDescriptions();
}

std::vector<ConnectorDescription> NodeFacadeRemote::getExternalEvents() const
{
    return nh_->getExternalEventDescriptions();
}

std::vector<ConnectorDescription> NodeFacadeRemote::getExternalSlots() const
{
    return nh_->getExternalSlotDescriptions();
}


NodeCharacteristics NodeFacadeRemote::getNodeCharacteristics() const
{
    return nh_->getVertex()->getNodeCharacteristics();
}

ConnectorPtr NodeFacadeRemote::getConnector(const UUID &id) const
{
    return remote_connectors_.at(id);
}



std::vector<param::ParameterPtr> NodeFacadeRemote::getParameters() const
{
    return nh_->getNode().lock()->getParameters();
}

param::ParameterPtr NodeFacadeRemote::getParameter(const std::string &name) const
{
    return nh_->getNode().lock()->getParameter(name);
}

bool NodeFacadeRemote::canStartStepping() const
{
    return nr_->canStartStepping();
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
    ss << (events_enabled ? "enabled" : "disabled") << ", ";
    ss << (nr_->canStartStepping() ? "canStartStepping" : "!canStartStepping");
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
