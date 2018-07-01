/// HEADER
#include <csapex/model/graph_facade_proxy.h>

/// PROJECT
#include <csapex/model/graph_facade_impl.h>
#include <csapex/model/graph/graph_proxy.h>
#include <csapex/model/graph/graph_impl.h>
#include <csapex/model/node_facade_proxy.h>
#include <csapex/model/node_facade_impl.h>
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

GraphFacadeProxy::GraphFacadeProxy(const SessionPtr& session, NodeFacadeProxyPtr remote_facade, GraphFacadeProxy* parent)
  : Proxy(session)
  , parent_(parent)
  , graph_channel_(session->openChannel(remote_facade->getAUUID()))
  , graph_handle_(remote_facade)
  , graph_(std::make_shared<GraphProxy>(graph_channel_, remote_facade))
  , uuid_(remote_facade->getAUUID())
  ,

/**
 * begin: initialize caches
 **/
#define HANDLE_ACCESSOR(_enum, type, function)
#define HANDLE_STATIC_ACCESSOR(_enum, type, function) has_##function##_(false),
#define HANDLE_DYNAMIC_ACCESSOR(_enum, signal, type, function) has_##function##_(false),
#define HANDLE_SIGNAL(_enum, signal)

#include <csapex/model/graph_facade_proxy_accessors.hpp>
  /**
   * end: initialize caches
   **/

  guard_(-1)
{
    if (parent_) {
        graph_->setParent(parent_->graph_, remote_facade->getAUUID());
    }

    observe(graph_->vertex_added, this, &GraphFacadeProxy::nodeAddedHandler);
    observe(graph_->vertex_removed, this, &GraphFacadeProxy::nodeRemovedHandler);

    observe(graph_->notification, notification);

    observe(graph_channel_->note_received, [this](const io::NoteConstPtr& note) {
        if (const std::shared_ptr<GraphFacadeNote const>& cn = std::dynamic_pointer_cast<GraphFacadeNote const>(note)) {
            switch (cn->getNoteType()) {
                case GraphFacadeNoteType::ChildAdded:
                    break;
                case GraphFacadeNoteType::ChildRemoved:
                    break;
                case GraphFacadeNoteType::ChildNodeFacadeAdded:
                    break;
                case GraphFacadeNoteType::ChildNodeFacadeRemoved:
                    break;
                case GraphFacadeNoteType::ForwardingConnectorAdded: {
                    ConnectorDescription cd = cn->getPayload<ConnectorDescription>(0);
                    createInternalConnector(cd);
                } break;
                case GraphFacadeNoteType::ForwardingConnectorRemoved: {
                    ConnectorDescription cd = cn->getPayload<ConnectorDescription>(0);
                    removeInternalConnector(cd);
                } break;
                case GraphFacadeNoteType::PauseChanged: {
                    paused(cn->getPayload<bool>(0));
                } break;
                case GraphFacadeNoteType::Notification: {
                    notification(cn->getPayload<Notification>(0));
                } break;

/**
 * begin: connect signals
 **/
#define HANDLE_ACCESSOR(_enum, type, function)
#define HANDLE_STATIC_ACCESSOR(_enum, type, function)
#define HANDLE_DYNAMIC_ACCESSOR(_enum, signal, type, function)                                                                                                                                         \
    case GraphFacadeNoteType::function##Changed: {                                                                                                                                                     \
        value_##function##_ = cn->getPayload<type>(0);                                                                                                                                                 \
        signal(value_##function##_);                                                                                                                                                                   \
    } break;
#define HANDLE_SIGNAL(_enum, signal)                                                                                                                                                                   \
    case GraphFacadeNoteType::_enum##Triggered: {                                                                                                                                                      \
        invokeSignal(signal, *cn);                                                                                                                                                                     \
    } break;

#include <csapex/model/graph_facade_proxy_accessors.hpp>
                    /**
                     * end: connect signals
                     **/
            }
        }
    });

    for (const ConnectorDescription& c : graph_handle_->getInternalConnectors()) {
        createInternalConnector(c);
    }

    observe(graph_->connection_added, connection_added);
    observe(graph_->connection_removed, connection_removed);

    observe(graph_->state_changed, state_changed);

    graph_->reload();
}

GraphFacadeProxy::~GraphFacadeProxy()
{
    guard_ = 0xDEADBEEF;
}

NodeFacadePtr GraphFacadeProxy::getNodeFacade() const
{
    return graph_handle_;
}

void GraphFacadeProxy::handleBroadcast(const BroadcastMessageConstPtr& message)
{
    if (auto graph_msg = std::dynamic_pointer_cast<GraphBroadcasts const>(message)) {
        switch (graph_msg->getBroadcastType()) {
            case GraphBroadcasts::GraphBroadcastType::None:
                break;
        }
    }
}

void GraphFacadeProxy::createInternalConnector(const ConnectorDescription& cd)
{
    graph_handle_->createConnectorProxy(cd);
    ConnectorPtr connector = graph_handle_->getConnector(cd.id);
    forwarding_connector_added(connector);
}
void GraphFacadeProxy::removeInternalConnector(const ConnectorDescription& cd)
{
    ConnectorPtr connector = graph_handle_->getConnector(cd.id);
    graph_handle_->removeConnectorProxy(cd);
    forwarding_connector_removed(connector);
}

void GraphFacadeProxy::createSubgraphFacade(NodeFacadePtr nf)
{
    NodeFacadeProxyPtr remote_facade = std::dynamic_pointer_cast<NodeFacadeProxy>(nf);
    apex_assert_hard(remote_facade);

    std::shared_ptr<GraphFacadeProxy> sub_graph_facade = std::make_shared<GraphFacadeProxy>(session_, remote_facade, this);
    children_[remote_facade->getUUID()] = sub_graph_facade;

    observe(sub_graph_facade->notification, notification);
    observe(sub_graph_facade->node_facade_added, child_node_facade_added);
    observe(sub_graph_facade->node_facade_removed, child_node_facade_removed);
    observe(sub_graph_facade->child_node_facade_added, child_node_facade_added);
    observe(sub_graph_facade->child_node_facade_removed, child_node_facade_removed);

    child_added(sub_graph_facade);
}

void GraphFacadeProxy::destroySubgraphFacade(NodeFacadePtr nf)
{
    auto pos = children_.find(nf->getUUID());
    if (pos != children_.end()) {
        std::shared_ptr<GraphFacadeProxy> subgraph = pos->second;
        children_.erase(pos);
        child_removed(subgraph);
    }
}

AUUID GraphFacadeProxy::getAbsoluteUUID() const
{
    return uuid_;
}

UUID GraphFacadeProxy::generateUUID(const std::string& prefix)
{
    return graph_channel_->request<UUID, GraphFacadeRequests>(GraphFacadeRequests::GraphFacadeRequestType::GenerateUUID, prefix);
}

GraphFacade* GraphFacadeProxy::getParent() const
{
    return parent_;
}

GraphFacadePtr GraphFacadeProxy::getSubGraph(const UUID& uuid)
{
    if (uuid.empty()) {
        throw std::logic_error("cannot get subgraph for empty UUID");
    }

    if (uuid.composite()) {
        GraphFacadePtr facade = children_[uuid.rootUUID()];
        return facade->getSubGraph(uuid.nestedUUID());
    } else {
        GraphFacadePtr facade = children_[uuid];
        return facade;
    }
}

NodeFacadePtr GraphFacadeProxy::findNodeFacade(const UUID& uuid) const
{
    return graph_->findNodeFacade(uuid);
}
NodeFacadePtr GraphFacadeProxy::findNodeFacadeNoThrow(const UUID& uuid) const noexcept
{
    return graph_->findNodeFacadeNoThrow(uuid);
}
NodeFacadePtr GraphFacadeProxy::findNodeFacadeForConnector(const UUID& uuid) const
{
    return graph_->findNodeFacadeForConnector(uuid);
}
NodeFacadePtr GraphFacadeProxy::findNodeFacadeForConnectorNoThrow(const UUID& uuid) const noexcept
{
    return graph_->findNodeFacadeForConnectorNoThrow(uuid);
}
NodeFacadePtr GraphFacadeProxy::findNodeFacadeWithLabel(const std::string& label) const
{
    return graph_->findNodeFacadeWithLabel(label);
}

ConnectorPtr GraphFacadeProxy::findConnector(const UUID& uuid)
{
    return graph_->findConnector(uuid);
}
ConnectorPtr GraphFacadeProxy::findConnectorNoThrow(const UUID& uuid) noexcept
{
    return graph_->findConnectorNoThrow(uuid);
}
bool GraphFacadeProxy::isConnected(const UUID& from, const UUID& to) const
{
    return graph_->isConnected(from, to);
}

ConnectionDescription GraphFacadeProxy::getConnection(const UUID& from, const UUID& to) const
{
    return graph_->getConnection(from, to);
}

ConnectionDescription GraphFacadeProxy::getConnectionWithId(int id) const
{
    return graph_->getConnectionWithId(id);
}

std::size_t GraphFacadeProxy::countNodes() const
{
    return graph_->countNodes();
}

int GraphFacadeProxy::getComponent(const UUID& node_uuid) const
{
    return graph_->getComponent(node_uuid);
}
int GraphFacadeProxy::getDepth(const UUID& node_uuid) const
{
    return graph_->getDepth(node_uuid);
}

GraphFacadeProxy* GraphFacadeProxy::getProxyParent() const
{
    return parent_;
}

void GraphFacadeProxy::nodeAddedHandler(graph::VertexPtr vertex)
{
    NodeFacadePtr facade = vertex->getNodeFacade();
    facade->notification.connect(notification);

    if (facade->isGraph()) {
        createSubgraphFacade(facade);
    }

    node_facade_added(facade);
}

void GraphFacadeProxy::nodeRemovedHandler(graph::VertexPtr vertex)
{
    NodeFacadePtr facade = vertex->getNodeFacade();
    if (facade->isGraph()) {
        destroySubgraphFacade(facade);
    }

    node_facade_removed(facade);
}

void GraphFacadeProxy::clearBlock()
{
    graph_channel_->sendRequest<GraphFacadeRequests>(GraphFacadeRequests::GraphFacadeRequestType::ClearBlock);
}

void GraphFacadeProxy::resetActivity()
{
    graph_channel_->sendRequest<GraphFacadeRequests>(GraphFacadeRequests::GraphFacadeRequestType::ResetActivity);
}

void GraphFacadeProxy::pauseRequest(bool pause)
{
    graph_channel_->sendRequest<GraphFacadeRequests>(GraphFacadeRequests::GraphFacadeRequestType::SetPause, pause);
}

std::vector<UUID> GraphFacadeProxy::enumerateAllNodes() const
{
    return graph_->getAllNodeUUIDs();
}
std::vector<ConnectionDescription> GraphFacadeProxy::enumerateAllConnections() const
{
    return graph_->enumerateAllConnections();
}

/**
 * begin: generate getters
 **/
#define HANDLE_ACCESSOR(_enum, type, function)                                                                                                                                                         \
    type GraphFacadeProxy::function() const                                                                                                                                                            \
    {                                                                                                                                                                                                  \
        return request<type, GraphFacadeRequests>(GraphFacadeRequests::GraphFacadeRequestType::_enum, uuid_.getAbsoluteUUID());                                                                        \
    }
#define HANDLE_STATIC_ACCESSOR(_enum, type, function)                                                                                                                                                  \
    type GraphFacadeProxy::function() const                                                                                                                                                            \
    {                                                                                                                                                                                                  \
        if (!has_##function##_) {                                                                                                                                                                      \
            cache_##function##_ = request<type, GraphFacadeRequests>(GraphFacadeRequests::GraphFacadeRequestType::_enum, uuid_.getAbsoluteUUID());                                                     \
            has_##function##_ = true;                                                                                                                                                                  \
        }                                                                                                                                                                                              \
        return cache_##function##_;                                                                                                                                                                    \
    }
#define HANDLE_DYNAMIC_ACCESSOR(_enum, signal, type, function)                                                                                                                                         \
    type GraphFacadeProxy::function() const                                                                                                                                                            \
    {                                                                                                                                                                                                  \
        if (!has_##function##_) {                                                                                                                                                                      \
            value_##function##_ = request<type, GraphFacadeRequests>(GraphFacadeRequests::GraphFacadeRequestType::_enum, uuid_.getAbsoluteUUID());                                                     \
            has_##function##_ = true;                                                                                                                                                                  \
        }                                                                                                                                                                                              \
        return value_##function##_;                                                                                                                                                                    \
    }
#define HANDLE_SIGNAL(_enum, signal)

#include <csapex/model/graph_facade_proxy_accessors.hpp>
/**
 * end: generate getters
 **/
