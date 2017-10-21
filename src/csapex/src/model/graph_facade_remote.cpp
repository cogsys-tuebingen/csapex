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
#include <csapex/io/protcol/graph_notes.h>
#include <csapex/model/graph/vertex.h>

/// SYSTEM
#include <iostream>

using namespace csapex;

GraphFacadeRemote::GraphFacadeRemote(SessionPtr session, AUUID uuid, GraphFacadeLocal& tmp_ref, GraphFacadeRemote* parent)
    : GraphFacade(tmp_ref.getNodeFacade()),
      Remote(session),
      parent_(parent),
      graph_(std::make_shared<GraphRemote>(session, uuid,
                                           *tmp_ref.getLocalGraph())),
      uuid_(uuid),
      tmp_ref_(tmp_ref),

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
    graph_channel_ = session->openChannel(uuid.getAbsoluteUUID());

    observe(graph_->vertex_added, delegate::Delegate<void(graph::VertexPtr)>(this, &GraphFacadeRemote::nodeAddedHandler));
    observe(graph_->vertex_removed, delegate::Delegate<void(graph::VertexPtr)>(this, &GraphFacadeRemote::nodeRemovedHandler));

    observe(graph_->notification, notification);

    observe(graph_channel_->note_received, [this](const io::NoteConstPtr& note){
        if(const std::shared_ptr<GraphNote const>& cn = std::dynamic_pointer_cast<GraphNote const>(note)) {

            /**
             * begin: connect signals
             **/
            #define HANDLE_ACCESSOR(_enum, type, function)
            #define HANDLE_STATIC_ACCESSOR(_enum, type, function)
            #define HANDLE_DYNAMIC_ACCESSOR(_enum, signal, type, function) \
                case GraphNoteType::function##Changed: \
                { \
                    value_##function##_ = cn->getPayload<type>(0);\
                    signal(value_##function##_); \
                } \
                break;
            #define HANDLE_SIGNAL(_enum, signal) \
                case GraphNoteType::_enum##Triggered: \
                { \
                    invokeSignal(signal, *cn); \
                } \
                break;

                #include <csapex/model/graph_facade_remote_accessors.hpp>
            /**
             * end: connect signals
             **/

        }
    });

    observe(tmp_ref.paused, paused);
    observe(tmp_ref.stopped, stopped);

    observe(tmp_ref.state_changed, state_changed);
    observe(tmp_ref.panic, panic);

//    observe(tmp_ref.forwarding_connector_added, forwarding_connector_added);
//    observe(tmp_ref.forwarding_connector_removed, forwarding_connector_removed);


    //TODO: these have to be translated

    observe(tmp_ref.connection_added, connection_added);
    observe(tmp_ref.connection_removed, connection_removed);

    observe(tmp_ref.child_added, child_added);
    observe(tmp_ref.child_removed, child_removed);

//    observe(tmp_ref.node_facade_added, node_facade_added);
//    observe(tmp_ref.node_facade_removed, node_facade_removed);

    observe(tmp_ref.child_node_facade_added, child_node_facade_added);
    observe(tmp_ref.child_node_facade_removed, child_node_facade_removed);



//    observe(tmp_ref.child_added, [this](GraphFacadePtr child) {
//        GraphFacadeLocalPtr tmp_child = std::dynamic_pointer_cast<GraphFacadeLocal>(child);
//        apex_assert_hard(tmp_child);

//        UUID uuid = child->getAbsoluteUUID();

//        std::shared_ptr<GraphFacadeRemote> sub_graph_facade = std::make_shared<GraphFacadeRemote>(session_, *tmp_child, this);

//        children_[uuid] = sub_graph_facade;

//        observe(sub_graph_facade->notification, notification);
//        observe(sub_graph_facade->node_facade_added, child_node_facade_added);
//        observe(sub_graph_facade->node_facade_removed, child_node_facade_removed);
//        observe(sub_graph_facade->child_node_facade_added, child_node_facade_added);
//        observe(sub_graph_facade->child_node_facade_removed, child_node_facade_removed);

//        child_added(sub_graph_facade);
//    });
//    observe(tmp_ref.child_removed, [this](GraphFacadePtr child) {
//        GraphFacadeLocalPtr tmp_child = std::dynamic_pointer_cast<GraphFacadeLocal>(child);
//        apex_assert_hard(tmp_child);

//        auto pos = children_.find(tmp_child->getAbsoluteUUID());
//        if(pos != children_.end()) {
//            child_removed(pos->second);
//            children_.erase(pos);
//        }
//    });

    observe(graph_->vertex_added, [this](graph::VertexPtr vertex) {
        node_facade_added(vertex->getNodeFacade());
    });

    observe(graph_->vertex_removed, [this](graph::VertexPtr vertex) {
        node_facade_removed(vertex->getNodeFacade());
    });

//    observe(tmp_ref.node_facade_added, [this](NodeFacadePtr node) {
//        NodeFacadeLocalPtr tmp_node = std::dynamic_pointer_cast<NodeFacadeLocal>(node);
//        apex_assert_hard(tmp_node);

//        NodeFacadePtr remote_node = findNodeFacade(node->getUUID());
//        node_facade_added(remote_node);
//    });
//    observe(tmp_ref.node_facade_removed, [this](NodeFacadePtr node) {
//        NodeFacadeLocalPtr tmp_node = std::dynamic_pointer_cast<NodeFacadeLocal>(node);
//        apex_assert_hard(tmp_node);

//        NodeFacadePtr remote_node = findNodeFacade(node->getUUID());
//        node_facade_removed(remote_node);
//    });
//    observe(tmp_ref.child_node_facade_added, [this](NodeFacadePtr node) {
//        NodeFacadeLocalPtr tmp_node = std::dynamic_pointer_cast<NodeFacadeLocal>(node);
//        apex_assert_hard(tmp_node);

//        NodeFacadePtr remote_node = findNodeFacade(node->getUUID());
//        child_node_facade_added(remote_node);
//    });
//    observe(tmp_ref.child_node_facade_removed, [this](NodeFacadePtr node) {
//        NodeFacadeLocalPtr tmp_node = std::dynamic_pointer_cast<NodeFacadeLocal>(node);
//        apex_assert_hard(tmp_node);

//        NodeFacadePtr remote_node = findNodeFacade(node->getUUID());
//        child_node_facade_removed(remote_node);
//    });
}

GraphFacadeRemote::~GraphFacadeRemote()
{
    guard_ = 0xDEADBEEF;
}

void GraphFacadeRemote::handleBroadcast(const BroadcastMessageConstPtr& message)
{
    if(auto graph_msg = std::dynamic_pointer_cast<GraphBroadcasts const>(message)) {
        switch(graph_msg->getBroadcastType()) {
        case GraphBroadcasts::GraphBroadcastType::GraphCreated:
        {
            std::cerr << "Remote graph " << graph_msg->getGraphUUID() << " has been created" << std::endl;
        }
            break;
        case GraphBroadcasts::GraphBroadcastType::GraphDestroyed:
        {
            std::cerr << "Remote graph " << graph_msg->getGraphUUID() << " has been destroyed" << std::endl;
        }
            break;
        case GraphBroadcasts::GraphBroadcastType::NodeCreated:
        {
            std::cerr << "Remote node " << graph_msg->getPayload<UUID>() << " has been created" << std::endl;
        }
            break;
        case GraphBroadcasts::GraphBroadcastType::NodeDestroyed:
        {
            std::cerr << "Remote node " << graph_msg->getPayload<UUID>() << " has been destroyed" << std::endl;
        }
            break;
        default:
            break;
        }
    }
}

AUUID GraphFacadeRemote::getAbsoluteUUID() const
{
    return tmp_ref_.getAbsoluteUUID();
}

UUID GraphFacadeRemote::generateUUID(const std::string& prefix)
{
    return tmp_ref_.generateUUID(prefix);
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
    return tmp_ref_.isConnected(from, to);
}

ConnectionInformation GraphFacadeRemote::getConnection(const UUID& from, const UUID& to) const
{
    return tmp_ref_.getConnection(from, to);
}

ConnectionInformation GraphFacadeRemote::getConnectionWithId(int id) const
{
    return tmp_ref_.getConnectionWithId(id);
}

std::size_t GraphFacadeRemote::countNodes() const
{
    return tmp_ref_.countNodes();
}

int GraphFacadeRemote::getComponent(const UUID& node_uuid) const
{
    return tmp_ref_.getComponent(node_uuid);
}
int GraphFacadeRemote::getDepth(const UUID& node_uuid) const
{
    return tmp_ref_.getDepth(node_uuid);
}

GraphFacadeRemote* GraphFacadeRemote::getRemoteParent() const
{
    return parent_;
}

void GraphFacadeRemote::nodeAddedHandler(graph::VertexPtr vertex)
{
    vertex->getNodeFacade()->notification.connect(notification);
}

void GraphFacadeRemote::nodeRemovedHandler(graph::VertexPtr vertex)
{
    // TODO: implement client server
}

void GraphFacadeRemote::stop()
{
    // TODO: implement client server
    tmp_ref_.stop();
}
void GraphFacadeRemote::clear()
{
    // TODO: implement client server
    tmp_ref_.clear();
}

void GraphFacadeRemote::clearBlock()
{
    // TODO: implement client server
    tmp_ref_.clearBlock();
}

void GraphFacadeRemote::resetActivity()
{
    // TODO: implement client server
    tmp_ref_.resetActivity();
}

bool GraphFacadeRemote::isPaused() const
{
    // TODO: implement client server
    return tmp_ref_.isPaused();
}
void GraphFacadeRemote::pauseRequest(bool pause)
{
    // TODO: implement client server
    tmp_ref_.pauseRequest(pause);
}


std::string GraphFacadeRemote::makeStatusString()
{
    // TODO: implement client server
    return tmp_ref_.makeStatusString();
}
std::vector<UUID> GraphFacadeRemote::enumerateAllNodes() const
{
    // TODO: implement client server
    return tmp_ref_.enumerateAllNodes();
}
std::vector<ConnectionInformation> GraphFacadeRemote::enumerateAllConnections() const
{
    // TODO: implement client server
    return tmp_ref_.enumerateAllConnections();
}


/**
 * begin: generate getters
 **/
#define HANDLE_ACCESSOR(_enum, type, function) \
type GraphFacadeRemote::function() const\
{\
    return request<type, GraphRequests>(GraphRequests::GraphRequestType::_enum, getUUID().getAbsoluteUUID());\
}
#define HANDLE_STATIC_ACCESSOR(_enum, type, function) \
type GraphFacadeRemote::function() const\
{\
    if(!has_##function##_) { \
        cache_##function##_ = request<type, GraphRequests>(GraphRequests::GraphRequestType::_enum, getUUID().getAbsoluteUUID());\
        has_##function##_ = true; \
    } \
    return cache_##function##_; \
}
#define HANDLE_DYNAMIC_ACCESSOR(_enum, signal, type, function) \
type GraphFacadeRemote::function() const\
{\
    if(!has_##function##_) { \
        value_##function##_ = request<type, GraphRequests>(GraphRequests::GraphRequestType::_enum, getUUID().getAbsoluteUUID());\
        has_##function##_ = true; \
    } \
    return value_##function##_; \
}
#define HANDLE_SIGNAL(_enum, signal)

#include <csapex/model/graph_facade_remote_accessors.hpp>
/**
 * end: generate getters
 **/

