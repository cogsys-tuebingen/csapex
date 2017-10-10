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
#include <csapex/io/protcol/node_notes.h>
#include <csapex/io/raw_message.h>
#include <csapex/model/connector_remote.h>
#include <csapex/io/channel.h>

/// SYSTEM
#include <iostream>
#include <sstream>

using namespace csapex;
using namespace csapex;

namespace detail
{
template <int pos, typename Arg, typename... Args>
struct ArgumentExtractor
{
    using type = typename ArgumentExtractor<pos-1, Args...>::type;
};


template <typename Arg, typename... Args>
struct ArgumentExtractor<0, Arg, Args...>
{
    using type = Arg;
};


template <int pos, typename Signature>
struct FunctionArgumentExtractor
{
    using type = Signature;
};

template <int pos, typename Result, typename... Args>
struct FunctionArgumentExtractor<pos, Result(Args...)>
{
    using type = typename ArgumentExtractor<pos, Args...>::type;
};
}

template <int pos, typename Signature>
struct FunctionArgumentExtractor
{
    using type = typename detail::FunctionArgumentExtractor<pos, Signature>::type;
};




template <int N, int arg_count>
struct SignalInvoker
{
public:
    template <typename Signature, typename... PartialArgs>
    static void doInvoke(csapex::slim_signal::Signal<Signature>& s, const NodeNote& note, PartialArgs... arguments)
    {
        using ArgN = typename FunctionArgumentExtractor<N, Signature>::type;
//        std::cerr << "invoke argument " << N << ": " << type2name(typeid(ArgN)) << std::endl;
        SignalInvoker<N + 1, arg_count - 1>::doInvoke(s, note, arguments..., note.getPayload<ArgN>(N));
    }
};

template <int N>
struct SignalInvoker<N, 0>
{

    template <typename... Args>
    static void doInvoke(csapex::slim_signal::Signal<void(Args...)>& s, const NodeNote& note, Args... arguments)
    {
//        std::cerr << "invoking signal with " << sizeof...(Args) << " arguments" << std::endl;
        s(arguments...);
    }
};


template <typename... Args>
static void invokeSignal(csapex::slim_signal::Signal<void(Args...)>& s, const NodeNote& note)
{
//    std::cerr << "trying to invoke signal with " << sizeof...(Args) << " arguments" << std::endl;
    SignalInvoker<0, sizeof...(Args)>::doInvoke(s, note);
}

NodeFacadeRemote::NodeFacadeRemote(SessionPtr session, AUUID uuid,
                                   NodeHandlePtr nh, NodeWorker* nw)
    : Remote(session),
      uuid_(uuid),
      nh_(nh), nw_(nw)
{
    node_channel_ = session->openChannel(uuid.getAbsoluteUUID());

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
                    std::cerr << "received node facade update: " << #function << " changed" << std::endl; \
                    auto new_value = boost::any_cast<type>(cn->getPayload()); \
                    value_##function##_ = new_value;\
                    signal(new_value); \
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
            case NodeNoteType::ConnectorCreatedTriggered:
            {
                ConnectorDescription info = cn->getPayload<ConnectorDescription>(0);
                createConnectorProxy(info.id);
            }
                break;
            case NodeNoteType::ConnectorRemovedTriggered:
            {
                ConnectorDescription info = cn->getPayload<ConnectorDescription>(0);
                removeConnectorProxy(info.id);
            }
                break;
            case NodeNoteType::ConnectionStartTriggered:
            {
                invokeSignal(connection_start, *cn);
            }
                break;
            case NodeNoteType::ConnectionDoneTriggered:
            {
                invokeSignal(connection_done, *cn);
            }
                break;
            case NodeNoteType::IntervalStartTriggered:
            {
                interval_start(this, cn->getPayload<ActivityType>(0), cn->getPayload<std::shared_ptr<const Interval>>(1));
            }
                break;
            case NodeNoteType::IntervalEndTriggered:
            {
                interval_end(this, cn->getPayload<std::shared_ptr<const Interval>>(0));
            }
                break;
            }
        }
    });

    for(const ConnectorDescription& c : getExternalConnectors()) {
        createConnectorProxy(c.id);
    }

    if(nw_) {
        connectNodeWorker();
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

void NodeFacadeRemote::connectNodeWorker()
{
    observe(nw_->notification, notification);
}

void NodeFacadeRemote::createConnectorProxy(const UUID &uuid)
{
    ConnectableOwnerPtr owner;
    std::shared_ptr<ConnectorRemote> proxy = std::make_shared<ConnectorRemote>(uuid, owner, session_);
    remote_connectors_[uuid] = proxy;
    connector_created(proxy->getDescription());
}
void NodeFacadeRemote::removeConnectorProxy(const UUID &uuid)
{
    auto pos = remote_connectors_.find(uuid);
    if(pos != remote_connectors_.end()) {
        auto proxy = pos->second;
        remote_connectors_.erase(pos);
        connector_removed(proxy->getDescription());
    }
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
#define HANDLE_SIGNAL(_enum, signal)

#include <csapex/model/node_facade_remote_accessors.hpp>
/**
 * end: generate getters
 **/
