/// HEADER
#include <csapex/model/node_facade_remote.h>

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
#include <csapex/model/connector_remote.h>
#include <csapex/model/node_characteristics.h>
#include <csapex/model/node_state.h>
#include <csapex/profiling/profiler_remote.h>
#include <csapex/utility/uuid_provider.h>
#include <csapex/utility/slim_signal_invoker.hpp>

/// SYSTEM
#include <iostream>

using namespace csapex;


NodeFacadeRemote::NodeFacadeRemote(Session &session, AUUID uuid)
    : Remote(session),
      uuid_(uuid),

      /**
       * begin: initialize caches
       **/
      #define HANDLE_ACCESSOR(_enum, type, function)
      #define HANDLE_STATIC_ACCESSOR(_enum, type, function) \
      has_##function##_(false),
      #define HANDLE_DYNAMIC_ACCESSOR(_enum, signal, type, function) \
      has_##function##_(false),
      #define HANDLE_SIGNAL(_enum, signal)

      #include <csapex/model/node_facade_remote_accessors.hpp>
      /**
       * end: initialize caches
       **/

      guard_(-1)
{
    node_channel_ = session.openChannel(uuid.getAbsoluteUUID());

    profiler_proxy_ = std::make_shared<ProfilerRemote>(node_channel_);

    observe(node_channel_->note_received, [this](const io::NoteConstPtr& note){
        if(const std::shared_ptr<NodeNote const>& cn = std::dynamic_pointer_cast<NodeNote const>(note)) {
            switch(cn->getNoteType()) {
            /**
             * begin: connect signals
             **/
            #define HANDLE_ACCESSOR(_enum, type, function)
            #define HANDLE_STATIC_ACCESSOR(_enum, type, function)
            #define HANDLE_DYNAMIC_ACCESSOR(_enum, signal, type, function) \
                case NodeNoteType::function##Changed: \
                { \
                    value_##function##_ = cn->getPayload<type>(0);\
                    signal(value_##function##_); \
                } \
                break;
            #define HANDLE_SIGNAL(_enum, signal) \
                case NodeNoteType::_enum##Triggered: \
                { \
                    invokeSignal(signal, *cn); \
                } \
                break;

                #include <csapex/model/node_facade_remote_accessors.hpp>
            /**
             * end: connect signals
             **/

            case NodeNoteType::ParameterAddedTriggered:
            {
                param::ParameterPtr p = cn->getPayload<param::ParameterPtr>(0)->clone<param::Parameter>();
                createParameterProxy(p);
                parameter_added(p);
            }
                break;
            case NodeNoteType::ParameterChangedTriggered:
            {
                param::ParameterPtr p = cn->getPayload<param::ParameterPtr>(0);
                auto pos = parameter_cache_.find(p->name());
                if(pos == parameter_cache_.end()) {
                    createParameterProxy(p);
                    parameter_added(p);

                } else {
                    param::ParameterPtr proxy = pos->second;
                    proxy->setValueFrom(*p);
                    parameter_changed(proxy);
                }

            }
                break;
            case NodeNoteType::ParameterRemovedTriggered:
            {
                param::ParameterPtr p = cn->getPayload<param::ParameterPtr>(0);
                std::string name = p->name();

                auto pos = parameter_cache_.find(name);
                if(pos != parameter_cache_.end()) {
                    parameter_removed(pos->second);
                    for(auto it = parameters_.begin(); it != parameters_.end(); ++it) {
                        if((*it)->name() == name) {
                            parameters_.erase(it);
                            break;
                        }
                    }
                    parameter_cache_.erase(pos);
                }
            }
                break;

            case NodeNoteType::ConnectorCreatedTriggered:
            {
                ConnectorDescription info = cn->getPayload<ConnectorDescription>(0);
                createConnectorProxy(info);
            }
                break;
            case NodeNoteType::ConnectorRemovedTriggered:
            {
                ConnectorDescription info = cn->getPayload<ConnectorDescription>(0);
                removeConnectorProxy(info);
            }
                break;
            case NodeNoteType::ConnectionStartTriggered:
            {
                invokeSignal(connection_start, *cn);
            }
                break;
            case NodeNoteType::ConnectionCreatedTriggered:
            {
                ConnectorDescription ci = cn->getPayload<ConnectorDescription>(0);
                connection_added(ci);
            }
                break;
            case NodeNoteType::ConnectionRemovedTriggered:
            {
                ConnectorDescription ci = cn->getPayload<ConnectorDescription>(0);
                connection_removed(ci);
            }
                break;
            case NodeNoteType::ProfilingStartTriggered:
            {
                start_profiling(this);
            }
                break;
            case NodeNoteType::ProfilingStopTriggered:
            {
                stop_profiling(this);
            }
                break;
            case NodeNoteType::IntervalStartTriggered:
            {
                interval_start(this, cn->getPayload<ActivityType>(0), cn->getPayload<std::shared_ptr<const Interval>>(1));
            }
                break;
            case NodeNoteType::IntervalEndTriggered:
            {
                std::shared_ptr<const Interval> interval = cn->getPayload<std::shared_ptr<const Interval>>(0);
                profiler_proxy_->updateInterval(interval);
                interval_end(this, interval);
            }
                break;
            case NodeNoteType::ErrorEvent:
            {
                bool e = cn->getPayload<bool>(0);
                std::string msg = cn->getPayload<std::string>(1);
                ErrorState::ErrorLevel level = cn->getPayload<ErrorState::ErrorLevel>(2);
                setError(e, msg, level);
            }
                break;
            case NodeNoteType::Notification:
            {
                notification(cn->getPayload<Notification>(0));
            }
                break;
            }
        }
    });

    auto params = node_channel_->request<std::vector<param::ParameterPtr>, NodeRequests>(NodeRequests::NodeRequestType::GetParameters);
    for(param::ParameterPtr& p : params) {
        createParameterProxy(p);
        parameter_added(p);
    }

    for(const ConnectorDescription& c : getExternalConnectors()) {
        createConnectorProxy(c);
    }
    for(const ConnectorDescription& c : getInternalConnectors()) {
        createConnectorProxy(c);
    }

    observe(remote_data_connection.first_connected, [this]() {
        node_channel_->sendRequest<NodeRequests>(NodeRequests::NodeRequestType::AddClient);
    });

    observe(remote_data_connection.last_disconnected, [this]() {
        node_channel_->sendRequest<NodeRequests>(NodeRequests::NodeRequestType::RemoveClient);
    });

    observe(node_channel_->raw_packet_received, [this](const RawMessageConstPtr& data) {
        remote_data_connection(data);
    });
}

NodeFacadeRemote::~NodeFacadeRemote()
{
    guard_ = 0xDEADBEEF;
}

void NodeFacadeRemote::handleBroadcast(const BroadcastMessageConstPtr& message)
{
    if(auto node_msg = std::dynamic_pointer_cast<NodeBroadcasts const>(message)) {
        switch(node_msg->getBroadcastType()) {
        default:
            break;
        }
    }
}

void NodeFacadeRemote::createConnectorProxy(const ConnectorDescription &cd)
{
    ConnectableOwnerPtr owner;
    std::shared_ptr<ConnectorRemote> proxy = std::make_shared<ConnectorRemote>(session_, cd.getAUUID(), owner, cd);
    remote_connectors_[cd.id] = proxy;
    connector_created(cd);
}

void NodeFacadeRemote::removeConnectorProxy(const ConnectorDescription &cd)
{
    auto pos = remote_connectors_.find(cd.id);
    if(pos != remote_connectors_.end()) {
        remote_connectors_.erase(pos);
        connector_removed(cd);
    }
}



UUID NodeFacadeRemote::getUUID() const
{
    if(uuid_.empty()) {
        return UUID::NONE;
    } else {
        return uuid_.id();
    }
}

AUUID NodeFacadeRemote::getAUUID() const
{
    return uuid_;
}

bool NodeFacadeRemote::isParameterInput(const UUID& id)
{
    auto pos = is_parameter_input_.find(id);
    if(pos == is_parameter_input_.end()) {
        bool result = node_channel_->request<bool, NodeRequests>(NodeRequests::NodeRequestType::IsParameterInput, id);
        is_parameter_input_[id] = result;
        return result;

    } else {
        return pos->second;
    }
}

bool NodeFacadeRemote::isParameterOutput(const UUID& id)
{
    auto pos = is_parameter_output_.find(id);
    if(pos == is_parameter_output_.end()) {
        bool result = node_channel_->request<bool, NodeRequests>(NodeRequests::NodeRequestType::IsParameterOutput, id);
        is_parameter_output_[id] = result;
        return result;

    } else {
        return pos->second;
    }
}


GraphPtr NodeFacadeRemote::getSubgraph() const
{
    apex_fail("Implement remote subgraph access!");
    return nullptr;
}


ConnectorPtr NodeFacadeRemote::getConnector(const UUID &id) const
{
    return remote_connectors_.at(id);
}
ConnectorPtr NodeFacadeRemote::getConnectorNoThrow(const UUID& id) const noexcept
{
    auto pos = remote_connectors_.find(id);
    if(pos == remote_connectors_.end()) {
        return nullptr;
    } else {
        return pos->second;
    }
}

ConnectorPtr NodeFacadeRemote::getParameterInput(const std::string& name) const
{
    return getConnector(UUIDProvider::makeDerivedUUID_forced(getUUID(), "in_" + name));
}

ConnectorPtr NodeFacadeRemote::getParameterOutput(const std::string& name) const
{
    return getConnector(UUIDProvider::makeDerivedUUID_forced(getUUID(), "out_" + name));
}



std::vector<param::ParameterPtr> NodeFacadeRemote::getParameters() const
{
    return parameters_;
}

param::ParameterPtr NodeFacadeRemote::getParameter(const std::string &name) const
{
    return parameter_cache_.at(name);
}

void NodeFacadeRemote::setProfiling(bool profiling)
{
    node_channel_->sendRequest<NodeRequests>(NodeRequests::NodeRequestType::SetProfiling, profiling);
}

NodeStatePtr NodeFacadeRemote::getNodeStateCopy() const
{
    return getNodeState()->clone<NodeState>();
}

ProfilerPtr NodeFacadeRemote::getProfiler()
{
    return profiler_proxy_;
}
std::string NodeFacadeRemote::getLoggerOutput(ErrorState::ErrorLevel level) const
{
    return node_channel_->request<std::string, NodeRequests>(NodeRequests::NodeRequestType::GetLoggerOutput, level);
}


bool NodeFacadeRemote::hasParameter(const std::string &name) const
{
    auto pos = parameter_cache_.find(name);
    return pos != parameter_cache_.end();
}

void NodeFacadeRemote::createParameterProxy(param::ParameterPtr proxy) const
{
    UUID uuid = UUIDProvider::makeUUID_forced(uuid_.getParent(), proxy->getUUID());
    proxy->setUUID(uuid);

    NodeFacadeRemote* self = const_cast<NodeFacadeRemote*>(this);
    proxy->parameter_changed.connect([self](param::Parameter* param){
        // request to set the parameter
        boost::any raw;
        param->get_unsafe(raw);
        CommandPtr change = std::make_shared<command::UpdateParameter>(param->getUUID(), raw);
        self->session_.write(change);
    });

    parameters_.push_back(proxy);
    parameter_cache_[proxy->name()] = proxy;
}


/**
 * begin: generate getters
 **/
#define HANDLE_ACCESSOR(_enum, type, function) \
type NodeFacadeRemote::function() const\
{\
    return request<type, NodeRequests>(NodeRequests::NodeRequestType::_enum, getUUID().getAbsoluteUUID());\
}
#define HANDLE_STATIC_ACCESSOR(_enum, type, function) \
type NodeFacadeRemote::function() const\
{\
    if(!has_##function##_) { \
        cache_##function##_ = request<type, NodeRequests>(NodeRequests::NodeRequestType::_enum, getUUID().getAbsoluteUUID());\
        has_##function##_ = true; \
    } \
    return cache_##function##_; \
}
#define HANDLE_DYNAMIC_ACCESSOR(_enum, signal, type, function) \
type NodeFacadeRemote::function() const\
{\
    if(!has_##function##_) { \
        value_##function##_ = request<type, NodeRequests>(NodeRequests::NodeRequestType::_enum, getUUID().getAbsoluteUUID());\
        has_##function##_ = true; \
    } \
    return value_##function##_; \
}
#define HANDLE_SIGNAL(_enum, signal)

#include <csapex/model/node_facade_remote_accessors.hpp>
/**
 * end: generate getters
 **/
