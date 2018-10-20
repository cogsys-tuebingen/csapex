/// HEADER
#include <csapex/model/node_facade_proxy.h>

/// PROJECT
#include <csapex/command/update_parameter.h>
#include <csapex/io/channel.h>
#include <csapex/io/protcol/node_broadcasts.h>
#include <csapex/io/protcol/node_notes.h>
#include <csapex/io/protcol/node_requests.h>
#include <csapex/io/protcol/parameter_changed.h>
#include <csapex/io/protcol/request_parameter.h>
#include <csapex/io/raw_message.h>
#include <csapex/io/session.h>
#include <csapex/model/connector_proxy.h>
#include <csapex/model/node_characteristics.h>
#include <csapex/model/node_state.h>
#include <csapex/profiling/profiler_proxy.h>
#include <csapex/utility/uuid_provider.h>
#include <csapex/utility/slim_signal_invoker.hpp>

/// SYSTEM
#include <iostream>

using namespace csapex;

NodeFacadeProxy::NodeFacadeProxy(const SessionPtr& session, AUUID uuid)
  : Proxy(session)
  , uuid_(uuid)
  ,

/**
 * begin: initialize caches
 **/
#define HANDLE_ACCESSOR(_enum, type, function)
#define HANDLE_STATIC_ACCESSOR(_enum, type, function) has_##function##_(false),
#define HANDLE_DYNAMIC_ACCESSOR(_enum, signal, type, function) has_##function##_(false),
#define HANDLE_SIGNAL(_enum, signal)

#include <csapex/model/node_facade_proxy_accessors.hpp>
  /**
   * end: initialize caches
   **/

  guard_(-1)
{
    node_channel_ = session->openChannel(uuid.getAbsoluteUUID());

    profiler_proxy_ = std::make_shared<ProfilerProxy>(node_channel_);

    state_proxy_ = node_channel_->request<NodeStatePtr, NodeRequests>(NodeRequests::NodeRequestType::GetNodeState);

    observe(node_channel_->note_received, [this](const io::NoteConstPtr& note) {
        if (const std::shared_ptr<NodeNote const>& cn = std::dynamic_pointer_cast<NodeNote const>(note)) {
            switch (cn->getNoteType()) {
/**
 * begin: connect signals
 **/
#define HANDLE_ACCESSOR(_enum, type, function)
#define HANDLE_STATIC_ACCESSOR(_enum, type, function)
#define HANDLE_DYNAMIC_ACCESSOR(_enum, signal, type, function)                                                                                                                                         \
    case NodeNoteType::function##Changed: {                                                                                                                                                            \
        value_##function##_ = cn->getPayload<type>(0);                                                                                                                                                 \
        signal(value_##function##_);                                                                                                                                                                   \
    } break;
#define HANDLE_SIGNAL(_enum, signal)                                                                                                                                                                   \
    case NodeNoteType::_enum##Triggered: {                                                                                                                                                             \
        invokeSignal(signal, *cn);                                                                                                                                                                     \
    } break;

#include <csapex/model/node_facade_proxy_accessors.hpp>
                /**
                 * end: connect signals
                 **/
                case NodeNoteType::NodeStateChanged: {
                    NodeStatePtr state = cn->getPayload<NodeStatePtr>(0);
                    apex_assert_hard(state);
                    *state_proxy_ = *state;
                } break;

                case NodeNoteType::ParameterAddedTriggered: {
                    param::ParameterPtr p = cn->getPayload<param::ParameterPtr>(0);
                    apex_assert_hard(p);
                    param::ParameterPtr proxy = p->cloneAs<param::Parameter>();
                    createParameterProxy(proxy);
                    parameter_added(proxy);
                } break;
                case NodeNoteType::ParameterChangedTriggered: {
                    param::ParameterPtr p = cn->getPayload<param::ParameterPtr>(0);
                    apex_assert_hard(p);
                    auto pos = parameter_cache_.find(p->name());
                    if (pos == parameter_cache_.end()) {
                        createParameterProxy(p);
                        parameter_added(p);

                    } else {
                        param::ParameterPtr proxy = pos->second;
                        proxy->cloneDataFrom(*p);
                        parameter_changed(proxy);
                    }

                } break;
                case NodeNoteType::ParameterRemovedTriggered: {
                    param::ParameterPtr p = cn->getPayload<param::ParameterPtr>(0);
                    std::string name = p->name();

                    auto pos = parameter_cache_.find(name);
                    if (pos != parameter_cache_.end()) {
                        parameter_removed(pos->second);
                        for (auto it = parameters_.begin(); it != parameters_.end(); ++it) {
                            if ((*it)->name() == name) {
                                parameters_.erase(it);
                                break;
                            }
                        }
                        parameter_cache_.erase(pos);
                    }
                } break;

                case NodeNoteType::ConnectorCreatedTriggered: {
                    ConnectorDescription info = cn->getPayload<ConnectorDescription>(0);
                    createConnectorProxy(info);
                } break;
                case NodeNoteType::ConnectorRemovedTriggered: {
                    ConnectorDescription info = cn->getPayload<ConnectorDescription>(0);
                    removeConnectorProxy(info);
                } break;
                case NodeNoteType::ConnectionStartTriggered: {
                    invokeSignal(connection_start, *cn);
                } break;
                case NodeNoteType::ConnectionCreatedTriggered: {
                    ConnectorDescription ci = cn->getPayload<ConnectorDescription>(0);
                    connection_added(ci);
                } break;
                case NodeNoteType::ConnectionRemovedTriggered: {
                    ConnectorDescription ci = cn->getPayload<ConnectorDescription>(0);
                    connection_removed(ci);
                } break;
                case NodeNoteType::ProfilingStartTriggered: {
                    start_profiling(this);
                } break;
                case NodeNoteType::ProfilingStopTriggered: {
                    stop_profiling(this);
                } break;
                case NodeNoteType::IntervalStartTriggered: {
                    interval_start(this, cn->getPayload<TracingType>(0), cn->getPayload<std::shared_ptr<const Interval>>(1));
                } break;
                case NodeNoteType::IntervalEndTriggered: {
                    std::shared_ptr<const Interval> interval = cn->getPayload<std::shared_ptr<const Interval>>(0);
                    profiler_proxy_->updateInterval(interval);
                    interval_end(this, interval);
                } break;
                case NodeNoteType::ErrorEvent: {
                    bool e = cn->getPayload<bool>(0);
                    std::string msg = cn->getPayload<std::string>(1);
                    ErrorState::ErrorLevel level = cn->getPayload<ErrorState::ErrorLevel>(2);
                    setError(e, msg, level);
                } break;
                case NodeNoteType::Notification: {
                    notification(cn->getPayload<Notification>(0));
                } break;
            }
        }
    });

    auto params = node_channel_->request<std::vector<param::ParameterPtr>, NodeRequests>(NodeRequests::NodeRequestType::GetParameters);
    for (param::ParameterPtr& p : params) {
        createParameterProxy(p);
        parameter_added(p);
    }

    for (const ConnectorDescription& c : getExternalConnectors()) {
        createConnectorProxy(c);
    }
    for (const ConnectorDescription& c : getInternalConnectors()) {
        createConnectorProxy(c);
    }

    observe(raw_data_connection.first_connected, [this]() { node_channel_->sendRequest<NodeRequests>(NodeRequests::NodeRequestType::AddClient); });

    observe(raw_data_connection.last_disconnected, [this]() { node_channel_->sendRequest<NodeRequests>(NodeRequests::NodeRequestType::RemoveClient); });

    observe(node_channel_->raw_packet_received, [this](const StreamableConstPtr& data) { raw_data_connection(data); });
}

NodeFacadeProxy::~NodeFacadeProxy()
{
    guard_ = 0xDEADBEEF;
    stopObserving();
}

bool NodeFacadeProxy::isProxy() const
{
    return true;
}

void NodeFacadeProxy::handleBroadcast(const BroadcastMessageConstPtr& message)
{
    if (auto node_msg = std::dynamic_pointer_cast<NodeBroadcasts const>(message)) {
        switch (node_msg->getBroadcastType()) {
            default:
                break;
        }
    }
}

void NodeFacadeProxy::createConnectorProxy(const ConnectorDescription& cd)
{
    ConnectableOwnerPtr owner;
    std::shared_ptr<ConnectorProxy> proxy = std::make_shared<ConnectorProxy>(session_, cd.getAUUID(), owner, cd);
    remote_connectors_[cd.id] = proxy;
    connector_created(cd);
}

void NodeFacadeProxy::removeConnectorProxy(const ConnectorDescription& cd)
{
    auto pos = remote_connectors_.find(cd.id);
    if (pos != remote_connectors_.end()) {
        remote_connectors_.erase(pos);
        connector_removed(cd);
    }
}

UUID NodeFacadeProxy::getUUID() const
{
    if (uuid_.empty()) {
        return UUID::NONE;
    } else {
        return uuid_.id();
    }
}

AUUID NodeFacadeProxy::getAUUID() const
{
    return uuid_;
}

bool NodeFacadeProxy::isParameterInput(const UUID& id)
{
    auto pos = is_parameter_input_.find(id);
    if (pos == is_parameter_input_.end()) {
        bool result = node_channel_->request<bool, NodeRequests>(NodeRequests::NodeRequestType::IsParameterInput, id);
        is_parameter_input_[id] = result;
        return result;

    } else {
        return pos->second;
    }
}

bool NodeFacadeProxy::isParameterOutput(const UUID& id)
{
    auto pos = is_parameter_output_.find(id);
    if (pos == is_parameter_output_.end()) {
        bool result = node_channel_->request<bool, NodeRequests>(NodeRequests::NodeRequestType::IsParameterOutput, id);
        is_parameter_output_[id] = result;
        return result;

    } else {
        return pos->second;
    }
}

GraphPtr NodeFacadeProxy::getSubgraph() const
{
    apex_fail("Implement remote subgraph access!");
    return nullptr;
}

ConnectorPtr NodeFacadeProxy::getConnector(const UUID& id) const
{
    return remote_connectors_.at(id);
}
ConnectorPtr NodeFacadeProxy::getConnectorNoThrow(const UUID& id) const noexcept
{
    auto pos = remote_connectors_.find(id);
    if (pos == remote_connectors_.end()) {
        return nullptr;
    } else {
        return pos->second;
    }
}

ConnectorPtr NodeFacadeProxy::getParameterInput(const std::string& name) const
{
    return getConnector(UUIDProvider::makeDerivedUUID_forced(getUUID(), "in_" + name));
}

ConnectorPtr NodeFacadeProxy::getParameterOutput(const std::string& name) const
{
    return getConnector(UUIDProvider::makeDerivedUUID_forced(getUUID(), "out_" + name));
}

std::vector<param::ParameterPtr> NodeFacadeProxy::getParameters() const
{
    return parameters_;
}

param::ParameterPtr NodeFacadeProxy::getParameter(const std::string& name) const
{
    return parameter_cache_.at(name);
}

void NodeFacadeProxy::setProfiling(bool profiling)
{
    node_channel_->sendRequest<NodeRequests>(NodeRequests::NodeRequestType::SetProfiling, profiling);
}

NodeStatePtr NodeFacadeProxy::getNodeState() const
{
    return state_proxy_;
}

NodeStatePtr NodeFacadeProxy::getNodeStateCopy() const
{
    return state_proxy_->cloneAs<NodeState>();
}

ProfilerPtr NodeFacadeProxy::getProfiler()
{
    return profiler_proxy_;
}
std::string NodeFacadeProxy::getLoggerOutput(ErrorState::ErrorLevel level) const
{
    return node_channel_->request<std::string, NodeRequests>(NodeRequests::NodeRequestType::GetLoggerOutput, level);
}

bool NodeFacadeProxy::hasParameter(const std::string& name) const
{
    auto pos = parameter_cache_.find(name);
    return pos != parameter_cache_.end();
}

void NodeFacadeProxy::createParameterProxy(param::ParameterPtr proxy) const
{
    UUID uuid = UUIDProvider::makeUUID_forced(uuid_.getParent(), proxy->getUUID());
    proxy->setUUID(uuid);

    NodeFacadeProxy* self = const_cast<NodeFacadeProxy*>(this);
    proxy->parameter_changed.connect([self](param::Parameter* param) {
        // request to set the parameter
        CommandPtr change = std::make_shared<command::UpdateParameter>(param->getUUID(), *param);
        self->session_->write(change);
    });

    parameters_.push_back(proxy);
    parameter_cache_[proxy->name()] = proxy;
}

/**
 * begin: generate getters
 **/
#define HANDLE_ACCESSOR(_enum, type, function)                                                                                                                                                         \
    type NodeFacadeProxy::function() const                                                                                                                                                             \
    {                                                                                                                                                                                                  \
        return request<type, NodeRequests>(NodeRequests::NodeRequestType::_enum, getUUID().getAbsoluteUUID());                                                                                         \
    }
#define HANDLE_STATIC_ACCESSOR(_enum, type, function)                                                                                                                                                  \
    type NodeFacadeProxy::function() const                                                                                                                                                             \
    {                                                                                                                                                                                                  \
        if (!has_##function##_) {                                                                                                                                                                      \
            cache_##function##_ = request<type, NodeRequests>(NodeRequests::NodeRequestType::_enum, getUUID().getAbsoluteUUID());                                                                      \
            has_##function##_ = true;                                                                                                                                                                  \
        }                                                                                                                                                                                              \
        return cache_##function##_;                                                                                                                                                                    \
    }
#define HANDLE_DYNAMIC_ACCESSOR(_enum, signal, type, function)                                                                                                                                         \
    type NodeFacadeProxy::function() const                                                                                                                                                             \
    {                                                                                                                                                                                                  \
        if (!has_##function##_) {                                                                                                                                                                      \
            value_##function##_ = request<type, NodeRequests>(NodeRequests::NodeRequestType::_enum, getUUID().getAbsoluteUUID());                                                                      \
            has_##function##_ = true;                                                                                                                                                                  \
        }                                                                                                                                                                                              \
        return value_##function##_;                                                                                                                                                                    \
    }
#define HANDLE_SIGNAL(_enum, signal)

#include <csapex/model/node_facade_proxy_accessors.hpp>
/**
 * end: generate getters
 **/
