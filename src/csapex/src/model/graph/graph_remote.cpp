/// HEADER
#include <csapex/model/graph/graph_remote.h>

/// PROJECT
#include <csapex/model/graph/graph_local.h>
#include <csapex/model/graph/vertex.h>
#include <csapex/model/connectable.h>
#include <csapex/model/connection_information.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_facade.h>
#include <csapex/model/node_facade_remote.h>
#include <csapex/model/node_facade_local.h>
#include <csapex/io/protcol/graph_notes.h>
#include <csapex/io/protcol/graph_requests.h>
#include <csapex/io/session.h>
#include <csapex/io/channel.h>
#include <csapex/utility/slim_signal_invoker.hpp>

/// SYSTEM
#include <iostream>

using namespace csapex;

GraphRemote::GraphRemote(io::ChannelPtr channel, NodeFacadeRemotePtr& node_facade)
    : graph_channel_(channel),

      /**
       * begin: initialize caches
       **/
      #define HANDLE_ACCESSOR(_enum, type, function)
      #define HANDLE_STATIC_ACCESSOR(_enum, type, function) \
      has_##function##_(false),
      #define HANDLE_DYNAMIC_ACCESSOR(_enum, signal, type, function) \
      has_##function##_(false),
      #define HANDLE_SIGNAL(_enum, signal)

      #include <csapex/model/graph_remote_accessors.hpp>
      /**
       * end: initialize caches
       **/

      nf_(node_facade)

{
    observe(graph_channel_->note_received, [this](const io::NoteConstPtr& note){
        if(const std::shared_ptr<GraphNote const>& cn = std::dynamic_pointer_cast<GraphNote const>(note)) {
            switch(cn->getNoteType())
            {

            case GraphNoteType::ConnectionAdded:
            {
                connectionAdded(cn->getPayload<ConnectionInformation>(0));
            }
                break;
            case GraphNoteType::ConnectionRemoved:
            {
                connectionRemoved(cn->getPayload<ConnectionInformation>(0));
            }
                break;
            case GraphNoteType::VertexAdded:
            {
                vertexAdded(cn->getPayload<UUID>(0));
            }
                break;
            case GraphNoteType::VertexRemoved:
            {
                vertexRemoved(cn->getPayload<UUID>(0));
            }
                break;

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

                #include <csapex/model/graph_remote_accessors.hpp>
            /**
             * end: connect signals
             **/
            }
        }
    });
}

GraphRemote::~GraphRemote()
{
}

void GraphRemote::reload()
{
    auto nodes = graph_channel_->request<std::vector<UUID>, GraphRequests>(GraphRequests::GraphRequestType::GetAllNodes);
    for(const UUID& id : nodes) {
        vertexAdded(id);
    }
    auto connections = graph_channel_->request<std::vector<ConnectionInformation>, GraphRequests>(GraphRequests::GraphRequestType::GetAllConnections);
    for(const ConnectionInformation& ci : connections) {
        connectionAdded(ci);
    }
}


void GraphRemote::vertexAdded(const UUID &id)
{
    AUUID auuid(makeUUID_forced(shared_from_this(), id.getFullName()).getAbsoluteUUID());
    std::shared_ptr<NodeFacadeRemote> remote_node_facade =
            std::make_shared<NodeFacadeRemote>(graph_channel_->getSession(), auuid);

    graph::VertexPtr remote_vertex = std::make_shared<graph::Vertex>(remote_node_facade);
    remote_vertices_.push_back(remote_vertex);
    vertex_added(remote_vertex);
}

void GraphRemote::vertexRemoved(const UUID& id)
{
    for(auto it = remote_vertices_.begin() ; it != remote_vertices_.end(); ++it) {
        graph::VertexPtr remote_vertex = *it;
        if(remote_vertex->getNodeFacade()->getUUID() == id) {
            vertex_removed(remote_vertex);
            remote_vertices_.erase(it);
            return;
        }
    }
}

void GraphRemote::connectionAdded(const ConnectionInformation& ci)
{
    edges_.push_back(ci);
    connection_added(ci);
}
void GraphRemote::connectionRemoved(const ConnectionInformation& ci)
{
    auto pos = std::find(edges_.begin(), edges_.end(), ci);
    if(pos != edges_.end()) {
        edges_.erase(pos);
        connection_removed(ci);
    }
}

AUUID GraphRemote::getAbsoluteUUID() const
{
    return nf_->getUUID().getAbsoluteUUID();
}

std::size_t GraphRemote::countNodes()
{
    return remote_vertices_.size();
}


int GraphRemote::getComponent(const UUID &node_uuid) const
{
    NodeFacadePtr nf = findNodeFacade(node_uuid);
    apex_assert_hard(nf);
    return nf->getNodeCharacteristics().component;
}

int GraphRemote::getDepth(const UUID &node_uuid) const
{
    NodeFacadePtr nf = findNodeFacade(node_uuid);
    apex_assert_hard(nf);
    return nf->getNodeCharacteristics().depth;
}



NodeFacadePtr GraphRemote::findNodeFacade(const UUID& uuid) const
{
    if(uuid.empty()) {
        return nf_;
    }
    NodeFacadePtr node_facade = findNodeFacadeNoThrow(uuid);
    if(node_facade) {
        return node_facade;
    }
    throw NodeFacadeNotFoundException(uuid.getFullName());
}
NodeFacadePtr GraphRemote::findNodeFacadeNoThrow(const UUID& uuid) const noexcept
{
    if(uuid.empty()) {
        return nf_;
    }
    if(uuid.composite()) {
        NodeFacadePtr root = findNodeFacadeNoThrow(uuid.rootUUID());
        if(root && root->isGraph()) {
            GraphPtr graph = root->getSubgraph();
            apex_assert_hard(graph);

            return graph->findNodeFacadeForConnectorNoThrow(uuid.nestedUUID());
        }

        return nullptr;

    } else {
        for(const auto& vertex : remote_vertices_) {
            NodeFacadePtr nf = vertex->getNodeFacade();
            if(nf->getUUID() == uuid) {
                return nf;
            }
        }
    }

    return nullptr;
}
NodeFacadePtr GraphRemote::findNodeFacadeForConnector(const UUID &uuid) const
{
    try {
        return findNodeFacade(uuid.parentUUID());

    } catch(const std::exception& e) {
        throw std::runtime_error(std::string("cannot find handle of connector \"") + uuid.getFullName());
    }
}
NodeFacadePtr GraphRemote::findNodeFacadeForConnectorNoThrow(const UUID &uuid) const noexcept
{
    return findNodeFacadeNoThrow(uuid.parentUUID());
}
NodeFacadePtr GraphRemote::findNodeFacadeWithLabel(const std::string& label) const
{
    for(const auto& vertex : remote_vertices_) {
        NodeFacadePtr nf = vertex->getNodeFacade();
        if(nf->getLabel() == label) {
            return nf;
        }
    }

    return nullptr;
}


std::vector<UUID> GraphRemote::getAllNodeUUIDs() const
{
    std::vector<UUID> uuids;
    for(const auto& vertex : remote_vertices_) {
        uuids.push_back(vertex->getUUID());
    }
    return uuids;
}
std::vector<NodeFacadePtr> GraphRemote::getAllNodeFacades()
{
    std::vector<NodeFacadePtr> node_facades;
    for(const graph::VertexPtr& vertex : remote_vertices_) {
        NodeFacadePtr nf = vertex->getNodeFacade();
        apex_assert_hard(std::dynamic_pointer_cast<NodeFacadeRemote>(nf));
        node_facades.push_back(nf);
    }

    return node_facades;
}

ConnectorPtr GraphRemote::findConnector(const UUID &uuid)
{
    ConnectorPtr res = findConnectorNoThrow(uuid);
    if(!res) {
        throw std::runtime_error(std::string("cannot find connector with UUID=") + uuid.getFullName());
    }
    return res;
}

ConnectorPtr GraphRemote::findConnectorNoThrow(const UUID &uuid) noexcept
{
    NodeFacadePtr owner = findNodeFacadeNoThrow(uuid.parentUUID());
    if(!owner) {
        return nullptr;
    }
    NodeFacadeRemotePtr owner_remote = std::dynamic_pointer_cast<NodeFacadeRemote>(owner);
    apex_assert_hard(owner_remote);

    return owner->getConnectorNoThrow(uuid);
}

ConnectionInformation GraphRemote::getConnection(const UUID& from, const UUID& to) const
{
    for(const ConnectionInformation& ci : edges_) {
        if(ci.from == from && ci.to == to) {
            return ci;
        }
    }
    throw std::runtime_error(from.getFullName() + " and " + to.getFullName() + " are not connected");
}
ConnectionInformation GraphRemote::getConnectionWithId(int id) const
{
    for(const ConnectionInformation& ci : edges_) {
        if(ci.id == id) {
            return ci;
        }
    }
    throw std::runtime_error(std::string("there is no connection with id ") + std::to_string(id));
}

bool GraphRemote::isConnected(const UUID &from, const UUID &to) const
{
    for(const ConnectionInformation& ci : edges_) {
        if(ci.from == from && ci.to == to) {
            return true;
        }
    }
    return false;
}

std::vector<ConnectionInformation> GraphRemote::enumerateAllConnections() const
{
    return edges_;
}

/**
 * begin: generate getters
 **/
#define HANDLE_ACCESSOR(_enum, type, function) \
type GraphRemote::function() const\
{\
    return request<type, GraphRequests>(GraphRequests::GraphRequestType::_enum, uuid_.getAbsoluteUUID());\
}
#define HANDLE_STATIC_ACCESSOR(_enum, type, function) \
type GraphRemote::function() const\
{\
    if(!has_##function##_) { \
        cache_##function##_ = request<type, GraphRequests>(GraphRequests::GraphRequestType::_enum, uuid_.getAbsoluteUUID());\
        has_##function##_ = true; \
    } \
    return cache_##function##_; \
}
#define HANDLE_DYNAMIC_ACCESSOR(_enum, signal, type, function) \
type GraphRemote::function() const\
{\
    if(!has_##function##_) { \
        value_##function##_ = request<type, GraphRequests>(GraphRequests::GraphRequestType::_enum, uuid_.getAbsoluteUUID());\
        has_##function##_ = true; \
    } \
    return value_##function##_; \
}
#define HANDLE_SIGNAL(_enum, signal)

#include <csapex/model/graph_remote_accessors.hpp>
/**
 * end: generate getters
 **/
