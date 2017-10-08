/// HEADER
#include <csapex/io/graph_server.h>

/// PROJECT
#include <csapex/model/graph_facade_local.h>
#include <csapex/model/node_facade_local.h>
#include <csapex/io/node_server.h>
#include <csapex/io/session.h>
#include <csapex/io/protcol/graph_broadcasts.h>
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
}


void GraphServer::startObserving(const GraphFacadeLocalPtr &graph)
{
    AUUID graph_id = graph->getAbsoluteUUID();

    observe(graph->node_facade_added, [this, graph_id](const NodeFacadePtr& node) {
        node_server_->startObserving(std::dynamic_pointer_cast<NodeFacadeLocal>(node));
        session_->sendBroadcast<GraphBroadcasts>(GraphBroadcasts::GraphBroadcastType::NodeCreated,
                                                 graph_id,
                                                 node->getUUID());

    });
    observe(graph->node_facade_removed, [this, graph_id](const NodeFacadePtr& node) {
        node_server_->stopObserving(std::dynamic_pointer_cast<NodeFacadeLocal>(node));
        session_->sendBroadcast<GraphBroadcasts>(GraphBroadcasts::GraphBroadcastType::NodeDestroyed,
                                                 graph_id,
                                                 node->getUUID());
    });
    observe(graph->child_added, [this](const GraphFacadePtr& graph) {
        startObserving(std::dynamic_pointer_cast<GraphFacadeLocal>(graph));
        session_->sendBroadcast<GraphBroadcasts>(GraphBroadcasts::GraphBroadcastType::GraphCreated,
                                                 graph->getAbsoluteUUID());
    });
    observe(graph->child_removed, [this](const GraphFacadePtr& graph) {
        startObserving(std::dynamic_pointer_cast<GraphFacadeLocal>(graph));
        session_->sendBroadcast<GraphBroadcasts>(GraphBroadcasts::GraphBroadcastType::GraphDestroyed,
                                                 graph->getAbsoluteUUID());
    });
}

void GraphServer::stopObserving(const GraphFacadeLocalPtr &graph)
{
    std::cerr << "stop serving graph: " << graph->getAbsoluteUUID() << std::endl;
}
