/// HEADER
#include <csapex/io/graph_server.h>

/// PROJECT
#include <csapex/model/graph_facade_impl.h>
#include <csapex/model/node_facade_impl.h>
#include <csapex/model/graph/graph_impl.h>
#include <csapex/model/graph/vertex.h>
#include <csapex/model/connection.h>
#include <csapex/model/connector.h>
#include <csapex/io/node_server.h>
#include <csapex/io/channel.h>
#include <csapex/io/session.h>
#include <csapex/io/protcol/graph_broadcasts.h>
#include <csapex/io/protcol/graph_notes.h>
#include <csapex/io/protcol/graph_facade_notes.h>
#include <csapex/io/node_server.h>

/// SYSTEM
#include <iostream>

using namespace csapex;

GraphServer::GraphServer(SessionPtr session)
    : session_(session)
{
    node_server_ = std::make_shared<NodeServer>(session);
}

GraphServer::~GraphServer()
{
    stopObserving();
}


void GraphServer::startObservingGraph(const GraphFacadeImplementationPtr &graph_facade)
{
    io::ChannelPtr channel = session_->openChannel(graph_facade->getAbsoluteUUID());

    for(const UUID& uuid : graph_facade->enumerateAllNodes()) {
        const auto& node = graph_facade->findNodeFacade(uuid);
        node_server_->startObservingNode(std::dynamic_pointer_cast<NodeFacadeImplementation>(node));

        if(node->isGraph()) {
            startObservingGraph(graph_facade->getLocalSubGraph(uuid));
        }
    }

    observe(graph_facade->node_facade_added, [this, channel](const NodeFacadePtr& node) {
        node_server_->startObservingNode(std::dynamic_pointer_cast<NodeFacadeImplementation>(node));
        channel->sendNote<GraphFacadeNote>(GraphFacadeNoteType::ChildNodeFacadeAdded, node->getAUUID());

    });
    observe(graph_facade->node_facade_removed, [this, channel](const NodeFacadePtr& node) {
        node_server_->stopObservingNode(std::dynamic_pointer_cast<NodeFacadeImplementation>(node));
        channel->sendNote<GraphFacadeNote>(GraphFacadeNoteType::ChildNodeFacadeRemoved, node->getAUUID());
    });
    observe(graph_facade->child_added, [this, channel](const GraphFacadePtr& graph) {
        startObservingGraph(std::dynamic_pointer_cast<GraphFacadeImplementation>(graph));
        channel->sendNote<GraphFacadeNote>(GraphFacadeNoteType::ChildAdded, graph->getAbsoluteUUID());

    });
    observe(graph_facade->child_removed, [this, channel](const GraphFacadePtr& graph) {
        startObservingGraph(std::dynamic_pointer_cast<GraphFacadeImplementation>(graph));
        channel->sendNote<GraphFacadeNote>(GraphFacadeNoteType::ChildRemoved, graph->getAbsoluteUUID());
    });

    observe(graph_facade->paused, [this, channel](bool pause) {
        channel->sendNote<GraphFacadeNote>(GraphFacadeNoteType::PauseChanged, pause);
    });
    observe(graph_facade->notification, [this, channel](Notification n){
        channel->sendNote<GraphFacadeNote>(GraphFacadeNoteType::Notification, n);
    });

    observe(graph_facade->forwarding_connector_added, [this, channel](const ConnectorPtr& c) {
        channel->sendNote<GraphFacadeNote>(GraphFacadeNoteType::ForwardingConnectorAdded, c->getDescription());

    });
    observe(graph_facade->forwarding_connector_removed, [this, channel](const ConnectorPtr& c) {
        channel->sendNote<GraphFacadeNote>(GraphFacadeNoteType::ForwardingConnectorRemoved, c->getDescription());
    });

    /**
     * begin: connect signals
     **/
    #define HANDLE_ACCESSOR(_enum, type, function)
    #define HANDLE_STATIC_ACCESSOR(_enum, type, function)
    #define HANDLE_DYNAMIC_ACCESSOR(_enum, signal, type, function) \
    observe(graph_facade->signal, [this, channel](const type& new_value){ \
        channel->sendNote<GraphFacadeNote>(GraphFacadeNoteType::function##Changed, new_value);  \
    });
    #define HANDLE_SIGNAL(_enum, signal) \
    observe(graph_facade->signal, [this, channel](){ \
        channel->sendNote<GraphFacadeNote>(GraphFacadeNoteType::_enum##Triggered);  \
    });

    #include <csapex/model/graph_facade_proxy_accessors.hpp>
    /**
     * end: connect signals
     **/


    GraphImplementationPtr graph = graph_facade->getLocalGraph();

    observe(graph->connection_added, [this, channel](const ConnectionDescription& ci){
        channel->sendNote<GraphNote>(GraphNoteType::ConnectionAdded, ci);
    });
    observe(graph->connection_removed, [this, channel](const ConnectionDescription& ci){
        channel->sendNote<GraphNote>(GraphNoteType::ConnectionRemoved, ci);
    });
    observe(graph->vertex_added, [this, channel](const graph::VertexPtr& vertex){
        channel->sendNote<GraphNote>(GraphNoteType::VertexAdded, vertex->getUUID());
    });
    observe(graph->vertex_removed, [this, channel](const graph::VertexPtr& vertex){
        channel->sendNote<GraphNote>(GraphNoteType::VertexRemoved, vertex->getUUID());
    });
    /**
     * begin: connect signals
     **/
    #define HANDLE_ACCESSOR(_enum, type, function)
    #define HANDLE_STATIC_ACCESSOR(_enum, type, function)
    #define HANDLE_DYNAMIC_ACCESSOR(_enum, signal, type, function) \
    observe(graph->signal, [this, channel](const type& new_value){ \
        channel->sendNote<GraphNote>(GraphNoteType::function##Changed, new_value);  \
    });
    #define HANDLE_SIGNAL(_enum, signal) \
    observe(graph->signal, [this, channel](){ \
        channel->sendNote<GraphNote>(GraphNoteType::_enum##Triggered);  \
    });

    #include <csapex/model/graph_proxy_accessors.hpp>
    /**
     * end: connect signals
     **/

}

void GraphServer::stopObservingGraph(const GraphFacadeImplementationPtr &graph)
{
}
