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
#include <csapex/io/protcol/node_broadcasts.h>
#include <csapex/io/raw_message.h>
#include <csapex/model/connector_remote.h>
#include <csapex/io/channel.h>

/// SYSTEM
#include <iostream>
#include <sstream>

using namespace csapex;
using namespace csapex;

NodeFacadeRemote::NodeFacadeRemote(SessionPtr session, AUUID uuid,
                                   NodeHandlePtr nh, NodeWorker* nw)
    : Remote(session),
      uuid_(uuid),
      nh_(nh), nw_(nw)
{

    if(nh_) {
        connectNodeHandle();
    }

    if(nw_) {
        connectNodeWorker();
    }

    node_channel_ = session->openChannel(uuid);

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
void NodeFacadeRemote::connectNodeHandle()
{
    observe(nh_->connector_created, [this](ConnectorPtr connector) {
        createConnectorProxy(connector->getUUID());
    });
    for(ConnectorPtr c : nh_->getExternalConnectors()) {
        createConnectorProxy(c->getUUID());
    }


    observe(nh_->connector_removed, [this](ConnectablePtr c) {
        connector_removed(c);
    });
    observe(nh_->node_state_changed, node_state_changed);


    observe(nh_->connection_done, [this](ConnectablePtr c) {
        connection_done(c);
    });
    observe(nh_->connection_start, [this](ConnectablePtr c) {
        connection_start(c);
    });

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

void NodeFacadeRemote::createConnectorProxy(const UUID &uuid)
{
    ConnectableOwnerPtr owner;
    std::shared_ptr<ConnectorRemote> proxy = std::make_shared<ConnectorRemote>(uuid, owner, session_);
    remote_connectors_[uuid] = proxy;
    connector_created(proxy);
}


UUID NodeFacadeRemote::getUUID() const
{
    return uuid_.id();
}

AUUID NodeFacadeRemote::getAUUID() const
{
    return uuid_;
}

bool NodeFacadeRemote::isParameterInput(const UUID& id)
{
    return node_channel_->request<bool, NodeRequests>(NodeRequests::NodeRequestType::IsParameterInput, id);
}

bool NodeFacadeRemote::isParameterOutput(const UUID& id)
{
    return node_channel_->request<bool, NodeRequests>(NodeRequests::NodeRequestType::IsParameterOutput, id);
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
    return nh_->getNode().lock()->getParameters();
}

param::ParameterPtr NodeFacadeRemote::getParameter(const std::string &name) const
{
    return nh_->getNode().lock()->getParameter(name);
}

void NodeFacadeRemote::setProfiling(bool profiling)
{
    node_channel_->sendRequest<NodeRequests>(NodeRequests::NodeRequestType::SetProfiling, profiling);
}

NodeStatePtr NodeFacadeRemote::getNodeState() const
{
    return nh_->getNodeState();
}
NodeStatePtr NodeFacadeRemote::getNodeStateCopy() const
{
    return nh_->getNodeStateCopy();
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
std::string NodeFacadeRemote::getLoggerOutput(ErrorState::ErrorLevel level) const
{
    return node_channel_->request<std::string, NodeRequests>(NodeRequests::NodeRequestType::GetLoggerOutput, level);
}

bool NodeFacadeRemote::hasParameter(const std::string &name) const
{
    if(auto node = nh_->getNode().lock()){
        return node->hasParameter(name);
    }
    throw std::runtime_error("tried to check a parameter from an invalid node");
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

#include <csapex/model/node_facade_remote_accessors.hpp>
/**
 * end: generate getters
 **/
