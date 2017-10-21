/// HEADER
#include <csapex/model/graph_facade_remote.h>

/// PROJECT
#include <csapex/model/graph_facade_local.h>
#include <csapex/model/graph/graph_remote.h>
#include <csapex/model/graph/graph_local.h>
#include <csapex/model/node_facade_remote.h>
#include <csapex/model/node_facade_local.h>
#include <csapex/io/channel.h>
#include <csapex/io/session.h>
#include <csapex/io/protcol/graph_broadcasts.h>
#include <csapex/io/protcol/graph_facade_notes.h>
#include <csapex/io/protcol/graph_facade_requests.h>
#include <csapex/model/graph/vertex.h>
#include <csapex/model/connection.h>
#include <csapex/utility/slim_signal_invoker.hpp>

/// SYSTEM
#include <iostream>

using namespace csapex;

GraphFacadeRemote::GraphFacadeRemote(Session& session, AUUID uuid, GraphFacadeRemote* parent)
    : GraphFacade(std::make_shared<NodeFacadeRemote>(session, uuid)),
      Remote(session),
      parent_(parent),
      graph_channel_(session.openChannel(uuid.getAbsoluteUUID())),
      graph_(std::make_shared<GraphRemote>(graph_channel_, uuid)),
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

      #include <csapex/model/graph_facade_remote_accessors.hpp>
      /**
       * end: initialize caches
       **/

      guard_(-1)
{
    observe(graph_->vertex_added, delegate::Delegate<void(graph::VertexPtr)>(this, &GraphFacadeRemote::nodeAddedHandler));
    observe(graph_->vertex_removed, delegate::Delegate<void(graph::VertexPtr)>(this, &GraphFacadeRemote::nodeRemovedHandler));

    observe(graph_->notification, notification);

    observe(graph_channel_->note_received, [this](const io::NoteConstPtr& note){
        if(const std::shared_ptr<GraphFacadeNote const>& cn = std::dynamic_pointer_cast<GraphFacadeNote const>(note)) {
            switch(cn->getNoteType())
            {
            case GraphFacadeNoteType::ChildAdded:
            {
                AUUID uuid = cn->getPayload<AUUID>(0);
                std::cerr << "Remote graph " << uuid << " has been created" << std::endl;
                //child_added();
            }
                break;
            case GraphFacadeNoteType::ChildRemoved:
            {
                AUUID uuid = cn->getPayload<AUUID>(0);
                std::cerr << "Remote graph " << uuid << " has been removed" << std::endl;
                //child_removed();
            }
                break;
            case GraphFacadeNoteType::ChildNodeFacadeAdded:
            {
                AUUID uuid = cn->getPayload<AUUID>(0);
                std::cerr << "Remote node " << uuid << " has been created" << std::endl;
                //child_node_facade_added();
            }
                break;
            case GraphFacadeNoteType::ChildNodeFacadeRemoved:
            {
                AUUID uuid = cn->getPayload<AUUID>(0);
                std::cerr << "Remote node " << uuid << " has been removed" << std::endl;
                //child_node_facade_removed();
            }
                break;
            case GraphFacadeNoteType::PauseChanged:
            {
                paused(cn->getPayload<bool>(0));
            }
                break;
            case GraphFacadeNoteType::Notification:
            {
                notification(cn->getPayload<Notification>(0));
            }
                break;

            /**
             * begin: connect signals
             **/
            #define HANDLE_ACCESSOR(_enum, type, function)
            #define HANDLE_STATIC_ACCESSOR(_enum, type, function)
            #define HANDLE_DYNAMIC_ACCESSOR(_enum, signal, type, function) \
                case GraphFacadeNoteType::function##Changed: \
                { \
                    value_##function##_ = cn->getPayload<type>(0);\
                    signal(value_##function##_); \
                } \
                break;
            #define HANDLE_SIGNAL(_enum, signal) \
                case GraphFacadeNoteType::_enum##Triggered: \
                { \
                    invokeSignal(signal, *cn); \
                } \
                break;

                #include <csapex/model/graph_facade_remote_accessors.hpp>
            /**
             * end: connect signals
             **/
            }
        }
    });

    //TODO: these have to be translated

    observe(graph_->connection_added, connection_added);
    observe(graph_->connection_removed, connection_removed);

//    observe(tmp_ref.child_added, child_added);
//    observe(tmp_ref.child_removed, child_removed);

//    observe(tmp_ref.child_node_facade_added, child_node_facade_added);
//    observe(tmp_ref.child_node_facade_removed, child_node_facade_removed);


    observe(graph_->state_changed, state_changed);

    observe(graph_->vertex_added, [this](graph::VertexPtr vertex) {
        node_facade_added(vertex->getNodeFacade());
    });

    observe(graph_->vertex_removed, [this](graph::VertexPtr vertex) {
        node_facade_removed(vertex->getNodeFacade());
    });

//    observe(graph_node_->forwarding_connector_added, forwarding_connector_added);
//    observe(graph_node_->forwarding_connector_removed, forwarding_connector_removed);
}

GraphFacadeRemote::~GraphFacadeRemote()
{
    guard_ = 0xDEADBEEF;
}

void GraphFacadeRemote::handleBroadcast(const BroadcastMessageConstPtr& message)
{
    if(auto graph_msg = std::dynamic_pointer_cast<GraphBroadcasts const>(message)) {
        switch(graph_msg->getBroadcastType()) {
        default:
            break;
        }
    }
}
void GraphFacadeRemote::createSubgraphFacade(NodeFacadePtr nf)
{
    NodeFacadeRemotePtr remote_facade = std::dynamic_pointer_cast<NodeFacadeRemote>(nf);
    apex_assert_hard(remote_facade);

    std::shared_ptr<GraphFacadeRemote> sub_graph_facade = std::make_shared<GraphFacadeRemote>(session_, nf->getAUUID(), this);
    children_[remote_facade->getUUID()] = sub_graph_facade;

    observe(sub_graph_facade->notification, notification);
    observe(sub_graph_facade->node_facade_added, child_node_facade_added);
    observe(sub_graph_facade->node_facade_removed, child_node_facade_removed);
    observe(sub_graph_facade->child_node_facade_added, child_node_facade_added);
    observe(sub_graph_facade->child_node_facade_removed, child_node_facade_removed);

    child_added(sub_graph_facade);
}

AUUID GraphFacadeRemote::getAbsoluteUUID() const
{
    return uuid_;
}

UUID GraphFacadeRemote::generateUUID(const std::string& prefix)
{
    return graph_channel_->request<UUID, GraphFacadeRequests>(GraphFacadeRequests::GraphFacadeRequestType::GenerateUUID, prefix);
}

GraphFacade* GraphFacadeRemote::getParent() const
{
    return parent_;
}

GraphFacade* GraphFacadeRemote::getSubGraph(const UUID &uuid)
{
    if(uuid.empty()) {
        throw std::logic_error("cannot get subgraph for empty UUID");
    }

    if(uuid.composite()) {
        GraphFacadePtr facade = children_[uuid.rootUUID()];
        return facade->getSubGraph(uuid.nestedUUID());
    } else {
        GraphFacadePtr facade = children_[uuid];
        return facade.get();
    }
}


NodeFacadePtr GraphFacadeRemote::findNodeFacade(const UUID& uuid) const
{
    return graph_->findNodeFacade(uuid);
}
NodeFacadePtr GraphFacadeRemote::findNodeFacadeNoThrow(const UUID& uuid) const noexcept
{
    return graph_->findNodeFacadeNoThrow(uuid);
}
NodeFacadePtr GraphFacadeRemote::findNodeFacadeForConnector(const UUID &uuid) const
{
    return graph_->findNodeFacadeForConnector(uuid);
}
NodeFacadePtr GraphFacadeRemote::findNodeFacadeForConnectorNoThrow(const UUID &uuid) const noexcept
{
    return graph_->findNodeFacadeForConnectorNoThrow(uuid);
}
NodeFacadePtr GraphFacadeRemote::findNodeFacadeWithLabel(const std::string& label) const
{
    return graph_->findNodeFacadeWithLabel(label);
}

ConnectorPtr GraphFacadeRemote::findConnector(const UUID &uuid)
{
    return graph_->findConnector(uuid);
}
ConnectorPtr GraphFacadeRemote::findConnectorNoThrow(const UUID &uuid) noexcept
{
    return graph_->findConnectorNoThrow(uuid);
}
bool GraphFacadeRemote::isConnected(const UUID& from, const UUID& to) const
{
    return graph_->isConnected(from, to);
}

ConnectionInformation GraphFacadeRemote::getConnection(const UUID& from, const UUID& to) const
{
    return graph_->getConnection(from, to);
}

ConnectionInformation GraphFacadeRemote::getConnectionWithId(int id) const
{
    return graph_->getConnectionWithId(id);
}

std::size_t GraphFacadeRemote::countNodes() const
{
    return graph_->countNodes();
}

int GraphFacadeRemote::getComponent(const UUID& node_uuid) const
{
    return graph_->getComponent(node_uuid);
}
int GraphFacadeRemote::getDepth(const UUID& node_uuid) const
{
    return graph_->getDepth(node_uuid);
}

GraphFacadeRemote* GraphFacadeRemote::getRemoteParent() const
{
    return parent_;
}

void GraphFacadeRemote::nodeAddedHandler(graph::VertexPtr vertex)
{
    NodeFacadePtr facade = vertex->getNodeFacade();
    facade->notification.connect(notification);

    if(facade->isGraph()) {
        std::cerr << "subgraph added: " << facade->getUUID() << std::endl;
        createSubgraphFacade(facade);
    }
}

void GraphFacadeRemote::nodeRemovedHandler(graph::VertexPtr vertex)
{
    // TODO: implement client server
}


void GraphFacadeRemote::clearBlock()
{
    graph_channel_->sendRequest<GraphFacadeRequests>(GraphFacadeRequests::GraphFacadeRequestType::ClearBlock);
}

void GraphFacadeRemote::resetActivity()
{
    graph_channel_->sendRequest<GraphFacadeRequests>(GraphFacadeRequests::GraphFacadeRequestType::ResetActivity);
}

void GraphFacadeRemote::pauseRequest(bool pause)
{
    graph_channel_->sendRequest<GraphFacadeRequests>(GraphFacadeRequests::GraphFacadeRequestType::SetPause, pause);
}


std::vector<UUID> GraphFacadeRemote::enumerateAllNodes() const
{
    return graph_->getAllNodeUUIDs();
}
std::vector<ConnectionInformation> GraphFacadeRemote::enumerateAllConnections() const
{
    return graph_->enumerateAllConnections();
}


/**
 * begin: generate getters
 **/
#define HANDLE_ACCESSOR(_enum, type, function) \
type GraphFacadeRemote::function() const\
{\
    return request<type, GraphFacadeRequests>(GraphFacadeRequests::GraphFacadeRequestType::_enum, uuid_.getAbsoluteUUID());\
}
#define HANDLE_STATIC_ACCESSOR(_enum, type, function) \
type GraphFacadeRemote::function() const\
{\
    if(!has_##function##_) { \
        cache_##function##_ = request<type, GraphFacadeRequests>(GraphFacadeRequests::GraphFacadeRequestType::_enum, uuid_.getAbsoluteUUID());\
        has_##function##_ = true; \
    } \
    return cache_##function##_; \
}
#define HANDLE_DYNAMIC_ACCESSOR(_enum, signal, type, function) \
type GraphFacadeRemote::function() const\
{\
    if(!has_##function##_) { \
        value_##function##_ = request<type, GraphFacadeRequests>(GraphFacadeRequests::GraphFacadeRequestType::_enum, uuid_.getAbsoluteUUID());\
        has_##function##_ = true; \
    } \
    return value_##function##_; \
}
#define HANDLE_SIGNAL(_enum, signal)

#include <csapex/model/graph_facade_remote_accessors.hpp>
/**
 * end: generate getters
 **/

